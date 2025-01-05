
set(source_files    
                    motor_control_from_pi.cpp
                    ESP32Serial.cpp
                    SCSCL.cpp
                    SCS.cpp
                    SCSerial.cpp
                    SMS_STS.cpp)

idf_component_register(SRCS ${source_files}
                       INCLUDE_DIRS "."
                       )
