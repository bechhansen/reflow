idf_component_register(
    SRCS
        "main.c"
        "temperature.c"
        "zigbee_plug.c"
        "plug_fsm.c"
        "plug_ctrl.c"
        "reflow_curve.c"
        "reflow_algo.c"
        "reflow_profile.c"
        "reflow_ctrl.c"
        "web_server.c"
        "wifi_manager.c"
        "ota_update.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        nvs_flash
        esp_driver_i2c
        esp_driver_uart
        esp_driver_usb_serial_jtag
        esp_http_server
        esp_http_client
        esp_https_ota
        app_update
        esp_app_format
        esp-tls
        mbedtls
        spiffs
        cjson
        esp_wifi
        esp_coex
        esp_event
        esp_netif
        ieee802154
        esp_timer
        mdns
)

# The web UI and default profiles are embedded in the firmware (gzipped) so an
# over-the-air update carries them; SPIFFS holds only the user's profiles.
idf_build_get_property(python PYTHON)
set(WEB_DIR      ${CMAKE_CURRENT_LIST_DIR}/../web)
set(PROFILES_DIR ${CMAKE_CURRENT_LIST_DIR}/../profiles)
set(EMBED_PY     ${CMAKE_CURRENT_LIST_DIR}/../tools/embed_web.py)
set(WEB_C        ${CMAKE_CURRENT_BINARY_DIR}/web_files.c)
file(GLOB WEB_SRC      CONFIGURE_DEPENDS ${WEB_DIR}/*)
file(GLOB PROFILES_SRC CONFIGURE_DEPENDS ${PROFILES_DIR}/*.json)
add_custom_command(
    OUTPUT  ${WEB_C}
    COMMAND ${python} ${EMBED_PY} ${WEB_DIR} ${PROFILES_DIR} ${WEB_C}
    DEPENDS ${EMBED_PY} ${WEB_SRC} ${PROFILES_SRC}
    COMMENT "Embedding web UI and default profiles"
    VERBATIM)
target_sources(${COMPONENT_LIB} PRIVATE ${WEB_C})
