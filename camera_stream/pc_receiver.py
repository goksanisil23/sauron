import io
import socket
import threading
import time

import cv2
import numpy as np
from PIL import Image

RASPBERRY_PI_IP = "192.168.1.180"


class ImageClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.running = True

    def start(self):
        # Start the socket thread
        threading.Thread(target=self.receive_images, daemon=True).start()

    def receive_images(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            print(f"Connected to server {self.host}:{self.port}")

            while self.running:
                start_time = time.time()
                # Receive the size of the incoming image
                size_data = self.recvall(4)
                if not size_data:
                    print("No size data received. Exiting.")
                    break
                image_size = int.from_bytes(size_data, "big")
                # print(f"image size: {image_size}")

                # Receive the image data based on the size
                image_data = self.recvall(image_size)
                if not image_data:
                    print("No image data received. Exiting.")
                    break
                recv_t = time.time()
                recv_time = time.time() - start_time
                # Convert bytes to PIL Image and then to OpenCV format

                image = Image.open(io.BytesIO(image_data))
                image = cv2.cvtColor(
                    np.array(image), cv2.COLOR_RGB2BGR
                )  # Convert RGB to BGR for OpenCV
                decode_time = time.time() - recv_t
                # print(f"Decode time: {decode_time:.6f} seconds")

                # # Display the image using OpenCV
                cv2.imshow("Stream", image)

                # # Break on 'q' key press
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    self.running = False
                    break
                total_time = time.time() - start_time
                print(f"receive time: {recv_time:.6f} seconds")
                print(f"decode time: {decode_time:.6f} seconds")
                print(f"Total time: {total_time:.6f} seconds")
                print(f"image size: {image_size}")

        except ConnectionRefusedError:
            print(f"Failed to connect to {self.host}:{self.port}")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            self.sock.close()
            cv2.destroyAllWindows()

    def recvall(self, n):
        """Helper function to receive n bytes or return None if EOF is hit"""
        data = bytearray()
        while len(data) < n:
            packet = self.sock.recv(n - len(data))
            if not packet:
                return None
            data.extend(packet)
        return data


def main():
    # Replace 'raspberry_pi_ip' with the actual IP address of your Raspberry Pi
    HOST = RASPBERRY_PI_IP  # e.g., '192.168.1.100'
    PORT = 8000
    client = ImageClient(HOST, PORT)
    client.start()
    try:
        while True:
            pass  # Keeps the main thread alive
    except KeyboardInterrupt:
        client.running = False
        print("Client stopped.")


if __name__ == "__main__":
    main()
