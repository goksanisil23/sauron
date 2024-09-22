
set(source_files    gox_sts_motor_main.cpp
                    ESP32Serial.cpp
                    SCSCL.cpp
                    SCS.cpp
                    SCSerial.cpp
                    SMS_STS.cpp)

idf_component_register(SRCS ${source_files}
                       INCLUDE_DIRS "."
                       )
