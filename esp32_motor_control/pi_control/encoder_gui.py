import tkinter as tk
from image_capture import ImageCapture
from PIL import ImageTk


class EncoderGUI:
    def __init__(self, serial_comm):
        self.root = tk.Tk()
        self.root.title("Encoder GUI")

        # Create labels to display encoder values
        self.cam_label = tk.Label(self.root, text="Camera Enc: 0", font=("Arial", 14))
        self.cam_label.pack(pady=10)
        self.base_label = tk.Label(self.root, text="Base Enc: 0", font=("Arial", 14))
        self.base_label.pack(pady=10)

        # Keep track of slider interaction
        self.is_adjusting_cam = False
        self.is_adjusting_base = False

        # Camera slider
        self.cam_slider = tk.Scale(
            self.root,
            from_=1600,
            to=2300,
            orient=tk.HORIZONTAL,
            length=300,
            label="Camera Servo",
            font=("Arial", 12),
        )
        self.cam_slider.pack(pady=10)
        self.cam_slider.bind("<ButtonPress-1>", self.on_cam_slider_press)
        self.cam_slider.bind("<ButtonRelease-1>", self.slider_released)

        # Base slider
        self.base_slider = tk.Scale(
            self.root,
            from_=1000,
            to=3000,
            orient=tk.HORIZONTAL,
            length=300,
            label="Base Servo",
            font=("Arial", 12),
        )
        self.base_slider.pack(pady=10)
        self.base_slider.bind("<ButtonPress-1>", self.on_base_slider_press)
        self.base_slider.bind("<ButtonRelease-1>", self.slider_released)

        # ---- Bind arrow keys ----
        self.root.bind("<Up>", self.on_arrow_key)
        self.root.bind("<Down>", self.on_arrow_key)
        self.root.bind("<Left>", self.on_arrow_key)
        self.root.bind("<Right>", self.on_arrow_key)

        # ----- Image view via ImageCapture class -----
        self.image_label = tk.Label(self.root)
        self.image_label.pack(side="bottom", pady=10)
        self.image_capture = ImageCapture()
        self.update_image()

        self.serial_comm = serial_comm

    def update_image(self):
        image = self.image_capture.capture_image()
        photo = ImageTk.PhotoImage(image)
        self.image_label.config(image=photo)
        self.image_label.image = photo
        self.root.after(30, self.update_image)  # Update image every 30 ms

    def update_encoder_values(self, cam_pos, base_pos):
        self.cam_label.config(text=f"Camera Enc: {cam_pos}")
        self.base_label.config(text=f"Base Enc: {base_pos}")

        # Only adjust sliders if user is NOT adjusting it
        if not self.is_adjusting_cam:
            self.cam_slider.set(cam_pos)
        if not self.is_adjusting_base:
            self.base_slider.set(base_pos)

    def slider_released(self, event):
        self.is_adjusting_base = False
        self.is_adjusting_cam = False
        cam_val = self.cam_slider.get()
        base_val = self.base_slider.get()
        # print(f"Sending cam_servo: {cam_val}, base_servo: {base_val}")
        self.serial_comm.write(f"VAL:cam={cam_val},base={base_val}\0".encode())

    def on_cam_slider_press(self, event):
        self.is_adjusting_cam = True

    def on_base_slider_press(self, event):
        self.is_adjusting_base = True

    def on_arrow_key(self, event):
        """Increment or decrement sliders based on arrow keys."""
        if event.keysym == "Up":
            self.serial_comm.write(b"DIR:up\0")
        elif event.keysym == "Down":
            self.serial_comm.write(b"DIR:down\0")
        elif event.keysym == "Left":
            self.serial_comm.write(b"DIR:left\0")
        elif event.keysym == "Right":
            self.serial_comm.write(b"DIR:right\0")

    def startMainLoop(self):
        self.root.mainloop()
