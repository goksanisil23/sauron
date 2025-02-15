import tkinter as tk

from PIL import ImageTk, ImageDraw, Image
from servo_constants import *
import numpy as np
import cv2
import time


def detect_and_draw_img(image, face_detector):
    boxes_list, points_list = face_detector.detect(image)
    for boxes, keypoints in zip(boxes_list, points_list):
        *bbox, conf_score = boxes
        face = Face(kps=keypoints, bbox=bbox, age=None, gender=None)
        draw_face_info(image, face)

    box = boxes_list[0]
    box_center = (int((box[0] + box[2]) / 2), int((box[1] + box[3]) / 2))
    return box_center


class EncoderGUI:
    def __init__(self, serial_comm):
        self.face_center = None

        self.cam_enc = None
        self.base_enc = None

        self.img_width = None
        self.img_height = None

        # Initialize the timer
        self.prev_time = time.time()

        self.root = tk.Tk()
        self.root.title("Encoder GUI")

        # Create labels to display encoder values
        self.cam_label = tk.Label(self.root, text="Camera Enc: 0", font=("Verdana", 14))
        self.cam_label.pack(pady=10)
        self.base_label = tk.Label(self.root, text="Base Enc: 0", font=("Verdana", 14))
        self.base_label.pack(pady=10)

        # Keep track of slider interaction
        self.is_adjusting_cam = False
        self.is_adjusting_base = False

        # Camera servo slider
        self.cam_slider = tk.Scale(
            self.root,
            from_=CAM_SERVO_MIN_MAX[0],
            to=CAM_SERVO_MIN_MAX[1],
            orient=tk.HORIZONTAL,
            length=300,
            label="Camera Servo",
            font=("Verdana", 12),
        )
        self.cam_slider.pack(pady=10)
        self.cam_slider.bind("<ButtonPress-1>", self.on_cam_slider_press)
        self.cam_slider.bind("<ButtonRelease-1>", self.slider_released)

        # Base servo slider
        self.base_slider = tk.Scale(
            self.root,
            from_=BASE_SERVO_MIN_MAX[0],
            to=BASE_SERVO_MIN_MAX[1],
            orient=tk.HORIZONTAL,
            length=300,
            label="Base Servo",
            font=("Verdana", 12),
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

        self.widget_width, self.widget_height = (
            self.image_label.winfo_width(),
            self.image_label.winfo_height(),
        )

        # Bind image click event
        self.image_label.bind("<Button-1>", self.on_image_click)

        self.serial_comm = serial_comm

    def update_image(self, captured_image):

        if self.img_width is None:
            self.img_width = captured_image.shape[1]
            self.img_height = captured_image.shape[0]

        # photo = ImageTk.PhotoImage(captured_image)
        photo = ImageTk.PhotoImage(Image.fromarray(captured_image[:, :, ::-1]))
        # fps = 1 / (time.time() - self.prev_time)
        # self.prev_time = time.time()
        # print(f"FPS: {fps:.2f}")
        self.image_label.config(image=photo)
        self.image_label.image = photo

        # self.root.after(15, self.update_image)  # Update image every 30 ms

    def update_encoder_values(self, cam_pos, base_pos):
        self.cam_enc = cam_pos
        self.base_enc = base_pos
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
        self.serial_comm.write(f"VAL:cam={cam_val},base={base_val}\0".encode())

    def on_cam_slider_press(self, event):
        self.is_adjusting_cam = True

    def on_base_slider_press(self, event):
        self.is_adjusting_base = True

    def on_image_click(self, event):
        print(f"Image clicked at: ({event.x}, {event.y})")

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
