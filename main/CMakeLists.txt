idf_component_register(SRCS "mesh_main.c"
                        "legacy_proto.c"
                        "ds18b20_node.c"
                        "tds_node.c"
                        "stack_monitor.c"
                        "legacy_root_sender.c"
                        "mesh_time_sync.c"
                        "log_time_vprintf.c"
                        "mesh_log_stream.c"
                    PRIV_REQUIRES esp_wifi esp_driver_gpio nvs_flash esp_adc driver 
                    INCLUDE_DIRS "." "include")
