#include "esp_log.h"
#include "lwip/sockets.h"
#include "ble_proxy.h"
#include <string.h>

static const char *TAG = "TCP_PROXY";
static volatile bool proxy_running = false;
static int server_sock = -1;
static int clients[4] = {-1,-1,-1,-1};

void tcp_forward_ble_data(uint8_t *data, uint16_t len) {
    for (int i=0;i<4;i++) {
        if (clients[i] >= 0) {
            int n = send(clients[i], data, len, MSG_NOSIGNAL);
            if (n < 0) {
                ESP_LOGW(TAG, "Client %d disconnected", i);
                close(clients[i]);
                clients[i] = -1;
            }
        }
    }
}

static void close_all(void){
    if (server_sock>=0) { close(server_sock); server_sock=-1; }
    for (int i=0;i<4;i++) if (clients[i]>=0){ close(clients[i]); clients[i]=-1; }
}

static void tcp_task(void *_) {
    // Create and bind server socket
    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0) {
        ESP_LOGE(TAG, "Failed to create server socket");
        proxy_running = false;
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(4403),
        .sin_addr.s_addr = INADDR_ANY
    };

    if (bind(server_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind to port 4403");
        close(server_sock);
        server_sock = -1;
        proxy_running = false;
        vTaskDelete(NULL);
        return;
    }

    listen(server_sock, 4);
    proxy_running = true;
    ESP_LOGI(TAG, "📡 TCP proxy listening on :4403 - Ready for Meshtastic apps");

    while (proxy_running) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(server_sock, &fds);
        int maxfd = server_sock;

        for (int i = 0; i < 4; i++) {
            if (clients[i] >= 0) {
                FD_SET(clients[i], &fds);
                if (clients[i] > maxfd) maxfd = clients[i];
            }
        }

        struct timeval tv = {.tv_sec = 1, .tv_usec = 0};
        int sel = select(maxfd + 1, &fds, NULL, NULL, &tv);
        if (sel < 0) break;
        if (sel == 0) continue; // timeout

        // Accept new clients
        if (FD_ISSET(server_sock, &fds)) {
            int c = accept(server_sock, NULL, NULL);
            if (c >= 0) {
                bool added = false;
                for (int i = 0; i < 4; i++) {
                    if (clients[i] < 0) {
                        clients[i] = c;
                        ESP_LOGI(TAG, "Client %d connected", i);
                        added = true;
                        break;
                    }
                }
                if (!added) {
                    ESP_LOGW(TAG, "Max clients reached, rejecting");
                    close(c);
                }
            }
        }

        // Handle client data
        for (int i = 0; i < 4; i++) {
            if (clients[i] >= 0 && FD_ISSET(clients[i], &fds)) {
                uint8_t buf[512];
                int n = recv(clients[i], buf, sizeof(buf), 0);
                if (n <= 0) {
                    ESP_LOGI(TAG, "Client %d disconnected", i);
                    close(clients[i]);
                    clients[i] = -1;
                    continue;
                }

                // Forward to BLE if connected
                if (ble_proxy_is_connected()) {
                    esp_err_t ret = ble_proxy_send_data(buf, n);
                    if (ret != ESP_OK) {
                        ESP_LOGW(TAG, "BLE send failed: %s", esp_err_to_name(ret));
                    }
                }
            }
        }
    }

    // Cleanup
    close_all();
    vTaskDelete(NULL);
}

void start_tcp_proxy(void) {
    if (proxy_running) {
        ESP_LOGW(TAG, "TCP proxy already running");
        return;
    }

    BaseType_t ret = xTaskCreate(tcp_task, "tcp_proxy", 4096, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TCP proxy task");
    }
}

void stop_tcp_proxy(void) {
    if (!proxy_running) {
        return;
    }

    ESP_LOGI(TAG, "Stopping TCP proxy...");
    proxy_running = false; // tcp_task will close sockets and exit
}