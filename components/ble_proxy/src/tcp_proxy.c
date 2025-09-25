#include "esp_log.h"
#include "lwip/sockets.h"
#include "ble_proxy.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "host/ble_gap.h"
#include "host/ble_att.h"
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#define MIN(a,b) ((a) < (b) ? (a) : (b))

static const char *TAG = "TCP_PROXY";
static volatile bool proxy_running = false;
static TaskHandle_t tcp_task_handle = NULL;
static int server_sock = -1;
// Reduce memory usage for ESP32-C3
#define TCP_BUFFER_SIZE 128  // Further reduced for C3
#define MAX_CLIENTS 2        // Reduce from 4 to 2
static int clients[MAX_CLIENTS] = {-1, -1};
static SemaphoreHandle_t clients_mutex = NULL;

// Thread-safe client management
static void lock_clients(void) {
    if (clients_mutex) {
        xSemaphoreTake(clients_mutex, portMAX_DELAY);
    }
}

static void unlock_clients(void) {
    if (clients_mutex) {
        xSemaphoreGive(clients_mutex);
    }
}

static void close_client(int idx) {
    if (idx >= 0 && idx < MAX_CLIENTS && clients[idx] >= 0) {
        close(clients[idx]);
        clients[idx] = -1;
        ESP_LOGI(TAG, "Client %d disconnected", idx);
    }
}

// Thread-safe BLE data forwarding
void tcp_forward_ble_data(uint8_t *data, uint16_t len) {
    if (!proxy_running || !data || len == 0) {
        ESP_LOGW(TAG, "Cannot forward: proxy=%d, data=%p, len=%d",
                proxy_running, data, len);
        return;
    }

    ESP_LOGI(TAG, "Forwarding %d bytes from BLE to TCP clients", len);
    ESP_LOG_BUFFER_HEX(TAG, data, len < 16 ? len : 16);

    lock_clients();
    int sent_count = 0;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i] >= 0) {
            int n = send(clients[i], data, len, MSG_NOSIGNAL);
            if (n < 0) {
                ESP_LOGW(TAG, "Client %d send failed: %s", i, strerror(errno));
                close_client(i);
            } else if (n != len) {
                ESP_LOGW(TAG, "Client %d partial send: %d/%d bytes", i, n, len);
            } else {
                ESP_LOGI(TAG, "Sent %d bytes to client %d", n, i);
                sent_count++;
            }
        }
    }
    if (sent_count == 0) {
        ESP_LOGW(TAG, "No clients to forward BLE data to!");
    }
    unlock_clients();
}

static void close_all_connections(void) {
    lock_clients();
    if (server_sock >= 0) {
        close(server_sock);
        server_sock = -1;
        ESP_LOGI(TAG, "Server socket closed");
    }

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i] >= 0) {
            close_client(i);
        }
    }
    unlock_clients();
}

static bool add_client(int client_fd) {
    lock_clients();
    bool added = false;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i] < 0) {
            clients[i] = client_fd;
            ESP_LOGI(TAG, "Client %d connected (fd=%d)", i, client_fd);
            added = true;
            break;
        }
    }
    unlock_clients();
    return added;
}

static void tcp_task(void *param) {
    ESP_LOGI(TAG, "TCP proxy starting (optimized for ESP32-C3)...");

    // Create mutex for client management
    clients_mutex = xSemaphoreCreateMutex();
    if (!clients_mutex) {
        ESP_LOGE(TAG, "Failed to create clients mutex");
        proxy_running = false;
        vTaskDelete(NULL);
        return;
    }

    // Initialize client array
    lock_clients();
    for (int i = 0; i < MAX_CLIENTS; i++) {
        clients[i] = -1;
    }
    unlock_clients();

    // Create server socket
    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0) {
        ESP_LOGE(TAG, "Failed to create server socket: %s", strerror(errno));
        proxy_running = false;
        vTaskDelete(NULL);
        return;
    }

    // Set socket options
    int opt = 1;
    if (setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        ESP_LOGW(TAG, "Failed to set SO_REUSEADDR: %s", strerror(errno));
    }

    // Set non-blocking mode for accept
    int flags = fcntl(server_sock, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(server_sock, F_SETFL, flags | O_NONBLOCK);
    }

    // Bind to address
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(4403),
        .sin_addr.s_addr = INADDR_ANY
    };

    if (bind(server_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind to port 4403: %s", strerror(errno));
        close(server_sock);
        server_sock = -1;
        proxy_running = false;
        vTaskDelete(NULL);
        return;
    }

    // Listen for connections
    if (listen(server_sock, MAX_CLIENTS) < 0) {
        ESP_LOGE(TAG, "Failed to listen: %s", strerror(errno));
        close(server_sock);
        server_sock = -1;
        proxy_running = false;
        vTaskDelete(NULL);
        return;
    }

    proxy_running = true;
    ESP_LOGI(TAG, "📡 TCP proxy listening on :4403 - Ready for Meshtastic apps");

    // Main proxy loop
    while (proxy_running) {
        fd_set read_fds, except_fds;
        FD_ZERO(&read_fds);
        FD_ZERO(&except_fds);

        FD_SET(server_sock, &read_fds);
        FD_SET(server_sock, &except_fds);
        int maxfd = server_sock;

        // Add client sockets to fd_set
        lock_clients();
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i] >= 0) {
                FD_SET(clients[i], &read_fds);
                FD_SET(clients[i], &except_fds);
                if (clients[i] > maxfd) {
                    maxfd = clients[i];
                }
            }
        }
        unlock_clients();

        // Wait for activity with timeout
        struct timeval tv = {.tv_sec = 1, .tv_usec = 0};
        int activity = select(maxfd + 1, &read_fds, NULL, &except_fds, &tv);

        if (activity < 0) {
            if (errno != EINTR) {
                ESP_LOGE(TAG, "Select error: %s", strerror(errno));
                break;
            }
            continue;
        }

        if (activity == 0) {
            continue; // Timeout - check proxy_running
        }

        // Handle new connections
        if (FD_ISSET(server_sock, &read_fds)) {
            struct sockaddr_in client_addr;
            socklen_t addr_len = sizeof(client_addr);
            int client_fd = accept(server_sock, (struct sockaddr*)&client_addr, &addr_len);

            if (client_fd >= 0) {
                if (!add_client(client_fd)) {
                    ESP_LOGW(TAG, "Max clients reached, rejecting connection");
                    close(client_fd);
                } else {
                    ESP_LOGI(TAG, "Client connected from %s:%d",
                             inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
                }
            } else if (errno != EWOULDBLOCK && errno != EAGAIN) {
                ESP_LOGW(TAG, "Accept failed: %s", strerror(errno));
            }
        }

        // Handle server socket exceptions
        if (FD_ISSET(server_sock, &except_fds)) {
            ESP_LOGE(TAG, "Server socket exception");
            break;
        }

        // Handle client data and exceptions
        lock_clients();
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i] < 0) continue;

            // Check for exceptions first
            if (FD_ISSET(clients[i], &except_fds)) {
                ESP_LOGW(TAG, "Client %d exception", i);
                close_client(i);
                continue;
            }

            // Handle client data with proper chunking
            if (FD_ISSET(clients[i], &read_fds)) {
                uint8_t buf[TCP_BUFFER_SIZE];
                int n = recv(clients[i], buf, sizeof(buf), 0);

                if (n > 0) {
                    ESP_LOGD(TAG, "TCP->BLE: %d bytes", n);

                    // Chunk for BLE MTU (important from NimBLE docs)
                    size_t offset = 0;
                    size_t mtu_size = 20;  // Default, will be updated

                    // Get actual MTU if available from BLE proxy
                    ble_connection_t conn_info;
                    if (ble_proxy_get_connection_info(&conn_info) == ESP_OK &&
                        conn_info.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                        struct ble_gap_conn_desc desc;
                        if (ble_gap_conn_find(conn_info.conn_handle, &desc) == 0) {
                            mtu_size = ble_att_mtu(conn_info.conn_handle) - 3;
                            if (mtu_size > 244) mtu_size = 244;  // Cap at max
                        }
                    }

                    while (offset < n && ble_proxy_is_connected()) {
                        size_t chunk = MIN(n - offset, mtu_size);
                        esp_err_t ret = ble_proxy_send_data(buf + offset, chunk);
                        if (ret != ESP_OK) {
                            ESP_LOGW(TAG, "BLE send failed at offset %d", offset);
                            break;
                        }
                        offset += chunk;
                        if (offset < n) {
                            vTaskDelay(pdMS_TO_TICKS(5));  // Small delay
                        }
                    }
                } else if (n == 0) {
                    ESP_LOGI(TAG, "Client %d disconnected normally", i);
                    close_client(i);
                } else {
                    ESP_LOGW(TAG, "Client %d recv error: %s", i, strerror(errno));
                    close_client(i);
                }
            }
        }
        unlock_clients();
    }

    // Cleanup
    ESP_LOGI(TAG, "TCP proxy shutting down...");
    close_all_connections();

    if (clients_mutex) {
        vSemaphoreDelete(clients_mutex);
        clients_mutex = NULL;
    }

    tcp_task_handle = NULL;
    ESP_LOGI(TAG, "TCP proxy task terminated");
    vTaskDelete(NULL);
}

void start_tcp_proxy(void) {
    if (proxy_running || tcp_task_handle != NULL) {
        ESP_LOGW(TAG, "TCP proxy already running");
        return;
    }

    ESP_LOGI(TAG, "Starting TCP proxy...");
    // Smaller stack for ESP32-C3 (from 8192 to 3072)
    BaseType_t ret = xTaskCreate(tcp_task, "tcp_proxy", 3072, NULL, 5, &tcp_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TCP proxy task");
        tcp_task_handle = NULL;
    }
}

void stop_tcp_proxy(void) {
    if (!proxy_running && tcp_task_handle == NULL) {
        return;
    }

    ESP_LOGI(TAG, "Stopping TCP proxy...");
    proxy_running = false;

    // Wait for task to finish (with timeout)
    if (tcp_task_handle) {
        for (int i = 0; i < 50 && tcp_task_handle != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (tcp_task_handle != NULL) {
            ESP_LOGW(TAG, "Force terminating TCP proxy task");
            vTaskDelete(tcp_task_handle);
            tcp_task_handle = NULL;
        }
    }

    ESP_LOGI(TAG, "TCP proxy stopped");
}