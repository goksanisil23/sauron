from picamera2 import Picamera2
from PIL import Image
import io


# Setup Picamera2 image capturer class
class ImageCapture:
    def __init__(self):
        self.picam2 = Picamera2()
        self.picam2.options["quality"] = 100
        # config = picam2.create_still_configuration(main={"size": (640, 480)})
        config = self.picam2.create_video_configuration(main={"size": (800, 600)})
        self.picam2.configure(config)
        self.picam2.start()

    def capture_image(self):
        stream = io.BytesIO()
        self.picam2.capture_file(stream, format="jpeg")
        # return stream.getvalue()
        return Image.open(stream)
