from picamera2 import Picamera2
from PIL import Image
import io


# Setup Picamera2 image capturer class
class ImageCapture:
    def __init__(self):
        self.picam2 = Picamera2()
        self.picam2.options["quality"] = 100
        config = self.picam2.create_video_configuration(main={"size": (800, 600)})
        self.picam2.configure(config)
        self.picam2.start()

    def capture_image(self):
        stream = io.BytesIO()
        self.picam2.capture_file(stream, format="jpeg")
        return Image.open(stream)


class ImageCaptureCv:
    def __init__(self):
        self.picam2 = Picamera2()
        self.picam2.configure(
            self.picam2.create_preview_configuration(
                main={"format": "RGB888", "size": (800, 600)}
            )
        )
        self.picam2.start()

    def capture_image(self):
        return self.picam2.capture_array()
