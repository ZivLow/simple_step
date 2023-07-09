idf_component_register(
                    SRC_DIRS "." "tasks"
                    INCLUDE_DIRS "include"
                    REQUIRES "driver"
                    PRIV_REQUIRES "TMC-API" "button" "as5600")
