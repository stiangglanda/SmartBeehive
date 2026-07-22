idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS ""
                       REQUIRES esp_driver_gpio esp_rom esp_driver_i2s esp_driver_i2c driver esp32_camera esp_wifi esp_event nvs_flash esp_netif esp_http_server mqtt esp_http_client)