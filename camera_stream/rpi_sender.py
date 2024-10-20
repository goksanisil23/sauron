# Raspberry Pi: Stream images over socket
import io
import socket
import time

from picamera2 import Picamera2

# Set up camera
picam2 = Picamera2()
picam2.options["quality"] = 95
# config = picam2.create_still_configuration(main={"size": (640, 480)})
config = picam2.create_video_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

# Socket setup
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(("0.0.0.0", 8000))
server_socket.listen(1)
print("Waiting for connection...")
connection, client_address = server_socket.accept()

try:
    print("Client connected: ", client_address)
    time.sleep(4)
    while True:
        start_time = time.time()
        stream = io.BytesIO()
        picam2.capture_file(stream, format="jpeg")
        # Get the image in bytes
        image_data = stream.getvalue()
        elapsed_time = time.time() - start_time
        print(f"Capture time: {elapsed_time:.6f} seconds")

        # Send size first to know how many bytes to expect
        connection.sendall(len(image_data).to_bytes(4, "big"))
        connection.sendall(image_data)
        elapsed_time = time.time() - start_time
        print(f"Total time: {elapsed_time:.6f} seconds")
        print(f"image size: {len(image_data)}")

        # Without the sleep, in video mode, receiver is delayed a lot
        time.sleep(0.1)

finally:
    connection.close()
    server_socket.close()
