
set(source_files    
                    # gox_sts_just_monitor.cpp
                    # gox_sts_motor_main.cpp
                    motor_control_from_pi.cpp
                    # serial_to_pi.cpp
                    ESP32Serial.cpp
                    SCSCL.cpp
                    SCS.cpp
                    SCSerial.cpp
                    SMS_STS.cpp)

idf_component_register(SRCS ${source_files}
                       INCLUDE_DIRS "."
                       )
