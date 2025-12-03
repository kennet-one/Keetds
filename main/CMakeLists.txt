idf_component_register(SRCS "ds18b20_node.c" "mesh_main.c"
                        "legacy_proto.c"
                        "ds18b20_node.c"
                        "tds_node.c"
                        "stack_monitor.c"
                    PRIV_REQUIRES esp_wifi esp_driver_gpio esp_driver_ledc nvs_flash esp_adc driver 
                    INCLUDE_DIRS "." "include")
