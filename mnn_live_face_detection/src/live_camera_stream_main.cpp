#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "LibCamera.h"

using namespace cv;

int main() {
  time_t start_time = time(0);
  int frame_count = 0;
  float lens_position = 100;
  float focus_step = 50;
  LibCamera cam;
  uint32_t width = 800;
  uint32_t height = 600;
  uint32_t stride;
  char key;
  int window_width = 800;
  int window_height = 600;

  if (width > window_width) {
    cv::namedWindow("libcamera-demo", cv::WINDOW_NORMAL);
    cv::resizeWindow("libcamera-demo", window_width, window_height);
  }

  int ret = cam.initCamera();
  cam.configureStill(width, height, formats::RGB888, 1,
                     libcamera::Orientation::Rotate0);
  ControlList controls_;
  int64_t frame_time = 1'000'000 / 30;
  // Set frame rate
  controls_.set(controls::FrameDurationLimits,
                libcamera::Span<const int64_t, 2>({frame_time, frame_time}));
  // Adjust the brightness of the output images, in the range -1.0 to 1.0
  controls_.set(controls::Brightness, 0.5);
  // Adjust the contrast of the output image, where 1.0 = normal contrast
  controls_.set(controls::Contrast, 2.);
  // Set the exposure time
  controls_.set(controls::ExposureTime, 20000);
  cam.set(controls_);
  if (!ret) {
    bool flag;
    LibcameraOutData frameData;
    cam.startCamera();
    cam.VideoStream(&width, &height, &stride);
    while (true) {
      flag = cam.readFrame(&frameData);
      if (!flag)
        continue;
      Mat im(height, width, CV_8UC3, frameData.imageData, stride);

      imshow("libcamera-demo", im);
      key = waitKey(1);

      frame_count++;
      if ((time(0) - start_time) >= 1) {
        printf("fps: %d\n", frame_count);
        frame_count = 0;
        start_time = time(0);
      }
      cam.returnFrameBuffer(frameData);
    }
    destroyAllWindows();
    cam.stopCamera();
  }
  cam.closeCamera();
  return 0;
}
