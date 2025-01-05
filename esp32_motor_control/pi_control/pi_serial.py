import time
import serial
import threading

# SERIAL_PORT = "/dev/serial0"
SERIAL_PORT = "/dev/ttyUSB0"

ser = serial.Serial(SERIAL_PORT, 921600, timeout=1)  # 115200


def read_serial():
    """Continuously reads from the serial port."""
    while True:
        if ser.in_waiting > 0:
            response = ser.readline()
            # print(f"Received: {response}")
            if response.startswith(b"cam_enc:") and response.endswith(b"\n"):
                parts = response.strip().split(b",")
                cam_pos = int(parts[0].split(b":")[1])
                base_pos = int(parts[1].split(b":")[1])
                print(f"Camera position: {cam_pos}, Base position: {base_pos}")
        time.sleep(0.01)  # Avoid busy-waiting


def write_serial():
    """Handles occasional user input."""
    while True:
        user_input = input("Enter command (w/s or 'exit' to quit): ").strip().lower()
        if user_input == "w":
            ser.write(b"up\0")
        elif user_input == "s":
            ser.write(b"down\0")
        if user_input == "a":
            ser.write(b"left\0")
        elif user_input == "d":
            ser.write(b"right\0")
        elif user_input == "exit":
            print("Exiting...")
            ser.close()
            break
        time.sleep(0.01)  # Add a small delay


# Create and start threads
read_thread = threading.Thread(target=read_serial, daemon=True)
write_thread = threading.Thread(target=write_serial)

read_thread.start()
write_thread.start()

write_thread.join()  # Wait for user input thread to finish before exiting
