# components/bt_proxy/CMakeLists.txt

set(srcs "src/bt_proxy.c")
set(includes "include")

# ESP32C3 doesn't support Bluedroid, only NimBLE
if ( (IDF_TARGET STREQUAL "esp32" OR IDF_TARGET STREQUAL "esp32s3")
     AND CONFIG_BT_BLUEDROID_ENABLED )

    idf_component_register(
        SRCS         ${srcs}
        INCLUDE_DIRS ${includes}
        PRIV_REQUIRES
            bt
            esp_common
            driver
            lwip
            freertos
            nvs_flash
        REQUIRES
            bt
    )

else()
    # Create empty component for ESP32C3
    idf_component_register(
        INCLUDE_DIRS ${includes}
    )
    message(STATUS "bt_proxy: skipped for ESP32C3 (no Bluedroid support)")
endif()