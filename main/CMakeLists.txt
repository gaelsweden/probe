idf_component_register(
    SRCS "app.cpp" "main.cpp" 
         "../components/LoRa/lora.c"
    INCLUDE_DIRS "" 
                 "../components/LoRa/"
    )
