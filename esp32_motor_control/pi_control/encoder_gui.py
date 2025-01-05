import tkinter as tk
from image_capture import ImageCapture
from PIL import ImageTk


class EncoderGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Encoder GUI")

        # Create labels to display encoder values
        self.cam_label = tk.Label(
            self.root, text="Camera Position: 0", font=("Arial", 14)
        )
        self.cam_label.pack(pady=10)

        self.base_label = tk.Label(
            self.root, text="Base Position: 0", font=("Arial", 14)
        )
        self.base_label.pack(pady=10)

        self.image_label = tk.Label(self.root)
        self.image_label.pack(side="bottom", pady=10)
        self.image_capture = ImageCapture()
        self.update_image()

    def update_image(self):
        image = self.image_capture.capture_image()
        photo = ImageTk.PhotoImage(image)
        self.image_label.config(image=photo)
        self.image_label.image = photo
        self.root.after(30, self.update_image)  # Update image every 30 ms

    def move_up(self):
        ser.write(b"up\0")

    def move_down(self):
        ser.write(b"down\0")

    def move_left(self):
        ser.write(b"left\0")

    def move_right(self):
        ser.write(b"right\0")

    def update_encoder_values(self, cam_pos, base_pos):
        self.cam_label.config(text=f"Camera Position: {cam_pos}")
        self.base_label.config(text=f"Base Position: {base_pos}")

    def startMainLoop(self):
        self.root.mainloop()
