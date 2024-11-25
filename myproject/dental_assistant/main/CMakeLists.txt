idf_component_register( SRCS "app_main.cpp" "./src/mouth_openness.cpp" "./src/mo_utils.cpp" "./src/servo_driver.c" "./src/camera_node.cpp" "./src/image_util.c" 
                        INCLUDE_DIRS "." "./include"
                        # EMBED_FILES "./html/index.html"
                    )