import time

import serial

SERIAL_PORT = "/dev/serial0"
# SERIAL_PORT = "/dev/ttyUSB0"

ser = serial.Serial(SERIAL_PORT, 115200, timeout=1)

# Send data
while True:

    user_input = input("Enter command: ").strip().lower()
    if user_input == "w":
        ser.write(b"up\n")
    elif user_input == "s":
        ser.write(b"down\n")

    # Read response
    response = ser.readline()
    # print(f"Received: {response.decode().strip()}")
    print(f"Received: {response}")

    time.sleep(0.1)  # Adjust as needed

ser.close()
