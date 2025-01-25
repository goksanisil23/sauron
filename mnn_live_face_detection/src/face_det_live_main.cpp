
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <pthread.h>
#include <queue>
#include <thread>

#include <opencv2/opencv.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "CameraHandler.hpp"
#include "UltraFace.h"

constexpr int32_t  kUltraFaceImgWidth  = 320;
constexpr int32_t  kUltraFaceImgHeight = 240;
constexpr uint32_t kImageCaptureWidth  = 800;
constexpr uint32_t kImageCaptureHeight = 600;
constexpr int      kDesiredFPS{30};

bool setThreadAffinity(const std::vector<int> &core_ids)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    for (const auto &core_id : core_ids)
    {
        CPU_SET(core_id, &cpuset);
    }

    pthread_t current_thread = pthread_self();
    int       result         = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
    if (result != 0)
    {
        std::cerr << "Error setting thread affinity: " << strerror(result) << "\n";
        return false;
    }
    return true;
}

int main(int argc, char **argv)
{
    // Init face model
    std::string mnn_path = "../model/version-RFB/RFB-320.mnn";
    UltraFace   ultraface(mnn_path, kUltraFaceImgWidth, kUltraFaceImgHeight, 4, 0.65);

    // Init camera
    CameraHandler camera(kImageCaptureWidth, kImageCaptureHeight, kDesiredFPS);
    if (camera.initCamera() != 0)
    {
        std::cerr << "Failed to initialize the camera" << std::endl;
        return -1;
    }

    // Shared resources
    cv::Mat                 shared_frame;
    cv::Mat                 frame_to_process;
    std::mutex              frame_mutex;
    std::condition_variable frame_cv;
    bool                    new_frame_available = false;
    std::atomic<bool>       running(true);
    std::vector<FaceInfo>   face_info;
    std::mutex              face_mtx; // Protects access to face_info

    // Detection thread
    std::thread detector(
        [&]()
        {
            if (!setThreadAffinity({2, 3}))
            {
                std::cerr << "Failed to set thread affinity for detector thread.\n";
            }
            while (running)
            {
                std::unique_lock<std::mutex> lock(frame_mutex);
                // Wait until a new frame is available or termination is signaled
                frame_cv.wait(lock, [&]() { return new_frame_available || !running; });

                if (!running)
                    break;

                // Move the frame to process and reset the flag
                frame_to_process    = shared_frame.clone();
                new_frame_available = false; // signaling consumption
                lock.unlock();

                // Perform face detection
                std::vector<FaceInfo> local_face_info;
                ultraface.detect(frame_to_process, local_face_info);

                // Update the shared face_info
                {
                    std::lock_guard<std::mutex> face_lock(face_mtx);
                    face_info = std::move(local_face_info);
                }
            }
        });

    if (!setThreadAffinity({0, 1}))
    {
        std::cerr << "Failed to set thread affinity for main thread.\n";
        return -1;
    }

    cv::Mat captured_frame;
    while (true)
    {
        if (!camera.readFrame(captured_frame))
            continue;

        // Update the shared frame
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            shared_frame        = captured_frame.clone();
            new_frame_available = true;
        }
        frame_cv.notify_one(); // Notify the detector thread

        // Retrieve the latest face_info
        std::vector<FaceInfo> current_faces;
        {
            std::lock_guard<std::mutex> lock(face_mtx);
            current_faces = face_info;
        }

        // Draw bounding boxes
        for (const auto &face : current_faces)
        {
            cv::rectangle(captured_frame,
                          cv::Point(static_cast<int>(face.x1), static_cast<int>(face.y1)),
                          cv::Point(static_cast<int>(face.x2), static_cast<int>(face.y2)),
                          cv::Scalar(0, 255, 0),
                          2);
        }

        cv::imshow("libcamera-demo", captured_frame);
        if (cv::waitKey(1) == 27) // ESC key to exit
            break;
    }
    // Clean up
    running = false;
    frame_cv.notify_one(); // Wake up the detector thread if waiting
    detector.join();
    cv::destroyAllWindows();

    return 0;
}
