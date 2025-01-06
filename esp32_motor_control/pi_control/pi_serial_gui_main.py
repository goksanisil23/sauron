import time
import serial
import threading
from encoder_gui import EncoderGUI


# SERIAL_PORT = "/dev/serial0"
SERIAL_PORT = "/dev/ttyUSB0"

ser = serial.Serial(SERIAL_PORT, 921600, timeout=1)  # 115200


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


# Create the main Tkinter window
gui = EncoderGUI(serial_comm=ser)

# Create and start threads
read_thread = threading.Thread(target=read_serial, args=(gui,), daemon=True)

read_thread.start()

# Start the Tkinter event loop
gui.startMainLoop()
