# components/bt_proxy/CMakeLists.txt

set(srcs "src/bt_proxy.c")
set(includes "include")

# Build this component ONLY on chips that ship Bluedroid (ESP32 / ESP32-S3)
# AND when Bluedroid is enabled in sdkconfig.
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
    message(STATUS "bt_proxy: skipped (target='${IDF_TARGET}', Bluedroid enabled='${CONFIG_BT_BLUEDROID_ENABLED}')")
endif()
