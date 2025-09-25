idf_component_register(
    SRCS "src/ble_proxy.c" "src/ble_connection.c" "src/tcp_proxy.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash bt freertos esp_timer lwip
    # Remove the PRIV_REQUIRES line - nimble is part of bt
)

# Force enable security manager
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    MYNEWT_VAL_BLE_SM=1
    MYNEWT_VAL_BLE_SM_LEGACY=1
    MYNEWT_VAL_BLE_SM_SC=1
    MYNEWT_VAL_BLE_STORE_CONFIG=1
    NIMBLE_BLE_SM=1
)