#include <chrono>
#include <opencv2/opencv.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pthread.h>

#include "UltraFace.h"

constexpr int32_t kUltraFaceImgWidth  = 320;
constexpr int32_t kUltraFaceImgHeight = 240;

int main(int argc, char **argv)
{
    // Init face model
    std::string mnn_path   = "../model/version-RFB/RFB-320.mnn";
    std::string image_path = "../imgs/11.jpg";

    cv::Mat frame_to_process = cv::imread(image_path);

    UltraFace ultraface(mnn_path, kUltraFaceImgWidth, kUltraFaceImgHeight, 4, 0.65);

    const int             num_iterations = 1000;
    std::vector<FaceInfo> faces;
    auto                  start = std::chrono::steady_clock::now();
    for (int i = 0; i < num_iterations; i++)
    {
        // run the inference 100 times and take the average inference time
        ultraface.detect(frame_to_process, faces);
    }
    auto                          end     = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Avg Inference Time: " << elapsed.count() / num_iterations << " s" << std::endl;

    for (const auto &face : faces)
    {
        cv::rectangle(frame_to_process,
                      cv::Point(static_cast<int>(face.x1), static_cast<int>(face.y1)),
                      cv::Point(static_cast<int>(face.x2), static_cast<int>(face.y2)),
                      cv::Scalar(0, 255, 0),
                      2);
    }

    cv::imshow("libcamera-demo", frame_to_process);
    cv::waitKey(0);

    return 0;
}