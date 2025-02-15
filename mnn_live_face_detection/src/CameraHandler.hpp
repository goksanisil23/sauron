#include "LibCamera.h"

class CameraHandler
{
  public:
    CameraHandler(const uint32_t image_capture_width, const uint32_t image_capture_height, const int32_t desired_fps)
        : img_capture_width_(image_capture_width), img_capture_height_(image_capture_height), desired_fps_(desired_fps),
          frame_count_(0), start_time_(time(0))
    {
    }
    ~CameraHandler()
    {
        stopCamera();
        closeCamera();
    };

    int initCamera()
    {
        int ret = cam_.initCamera();
        if (ret)
            return ret;

        cam_.configureStill(
            img_capture_width_, img_capture_height_, formats::RGB888, 1, libcamera::Orientation::Rotate0);

        int64_t frame_time = 1'000'000 / desired_fps_; // FPS
        controls_.set(controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>({frame_time, frame_time}));
        controls_.set(controls::Brightness, 0.7);
        controls_.set(controls::Contrast, 2.0);
        controls_.set(controls::ExposureTime, 40'000);
        cam_.set(controls_);

        cam_.startCamera();
        cam_.VideoStream(&stream_width_, &stream_height_, &stream_stride_);

        return 0; // success
    }

    bool readFrame(cv::Mat &frame)
    {
        LibcameraOutData frameData;
        bool             ok = cam_.readFrame(&frameData);
        if (!ok)
            return false;

        frame = cv::Mat(stream_height_, stream_width_, CV_8UC3, frameData.imageData, stream_stride_).clone();
        cam_.returnFrameBuffer(frameData);

        // Optional: Simple FPS counter
        frame_count_++;
        if ((time(0) - start_time_) >= 1)
        {
            std::cout << "fps: " << frame_count_ << std::endl;
            frame_count_ = 0;
            start_time_  = time(0);
        }
        return true;
    }

    void stopCamera()
    {
        std::cout << "stopping camera" << std::endl;
        cam_.stopCamera();
    }
    void closeCamera()
    {
        std::cout << "Closing Camera" << std::endl;
        cam_.closeCamera();
    }

  private:
    LibCamera              cam_;
    libcamera::ControlList controls_;

    uint32_t img_capture_width_;
    uint32_t img_capture_height_;
    int32_t  desired_fps_;

    uint32_t stream_width_  = 0;
    uint32_t stream_height_ = 0;
    uint32_t stream_stride_ = 0;

    int    frame_count_{0};
    time_t start_time_{time(0)};
};
