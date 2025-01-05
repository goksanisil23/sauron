import time
import serial
import threading
import termios
import sys
import tty
from encoder_gui import EncoderGUI


# SERIAL_PORT = "/dev/serial0"
SERIAL_PORT = "/dev/ttyUSB0"

ser = serial.Serial(SERIAL_PORT, 921600, timeout=1)  # 115200


def get_key():
    """Reads a single key press without requiring Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def read_serial(gui):
    """Continuously reads from the serial port."""
    while True:
        if ser.in_waiting > 0:
            response = ser.readline()
            # print(f"Received: {response}")
            if response.startswith(b"cam_enc:") and response.endswith(b"\n"):
                parts = response.strip().split(b",")
                cam_pos = int(parts[0].split(b":")[1])
                base_pos = int(parts[1].split(b":")[1])
                # print(f"Camera position: {cam_pos}, Base position: {base_pos}")
                gui.update_encoder_values(cam_pos, base_pos)
        time.sleep(0.01)  # Avoid busy-waiting


def write_serial():
    """Handles occasional user input."""
    while True:
        key = get_key()

        if key == "w":
            ser.write(b"up\0")
        elif key == "a":
            ser.write(b"left\0")
        elif key == "s":
            ser.write(b"down\0")
        elif key == "d":
            ser.write(b"right\0")
        elif key == "x":  # Exit on 'x'
            break

        time.sleep(0.01)  # Add a small delay


# Create the main Tkinter window
gui = EncoderGUI()

# Create and start threads
read_thread = threading.Thread(target=read_serial, args=(gui,), daemon=True)
write_thread = threading.Thread(target=write_serial)

read_thread.start()
write_thread.start()

# Start the Tkinter event loop
gui.startMainLoop()

write_thread.join()  # Wait for user input thread to finish before exiting
