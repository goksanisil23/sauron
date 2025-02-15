## Pan & Tilt Camera Hardware Setup
This repo contains pan/tilt camera sensing and control applications, currently using Waveshare ST3215 as pan & tilt servos, and a raspberry-pi camera.

# Setup
- Follow the [video](https://youtu.be/SbexKaE8EFY?si=_NPguVa-FlvVz_ys) to assemble the pan-tilt camera with Waveshare General Driver For Robotics MCU.

- Initially, both pan and tilt servos have ID 1. To change that, disconnect the pan-servo, and run '.../.arduino15/libraries/SCServo/examples/STSCL/ProgramEprom/ProgramEprom.ino' which sets its ID to 2.



## Documents
- ST312 Manual : https://download.kamami.pl/p1181056-ST3215_Servo_User_Manual.pdf
- SC Servo driver original library: https://files.waveshare.com/upload/7/78/SCServo.rar
    - This repo contains a modified version of it that replaces Arduino dependent HardwareSerial with ESP32 serial:
    https://github.com/9a4gl/ESP32Serial/tree/master
- ESP-IDF setup: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html#get-started-prerequisites
- ESP-IDF c++: https://github.com/espressif/esp-idf-cxx/tree/main


# References
- Face detection module: https://github.com/Linzaer/Ultra-Light-Fast-Generic-Face-Detector-1MB/tree/master/MNN
