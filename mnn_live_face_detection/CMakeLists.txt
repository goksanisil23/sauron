cmake_minimum_required(VERSION 3.10)
project(face_det_realtime)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH})

find_package(PkgConfig REQUIRED)
pkg_check_modules(CAMERA REQUIRED libcamera)

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
	message("Found OpenCV")
	message("Includes:" ${OpenCV_INCLUDE_DIRS})
endif(OpenCV_FOUND)

include_directories(
        mnn/include
        src
        "${CAMERA_INCLUDE_DIRS}"
)
set(LIBCAMERA_LIBRARIES "${LIBCAMERA_LIBRARY}" "${LIBCAMERA_BASE_LIBRARY}")

find_library(LIBCAMERA_LIBRARY libcamera.so REQUIRED)
find_library(LIBCAMERA_BASE_LIBRARY libcamera-base.so REQUIRED)
message(STATUS ${CAMERA_INCLUDE_DIRS})
message(STATUS ${LIBCAMERA_LIBRARY})
message(STATUS ${LIBCAMERA_BASE_LIBRARY})

link_directories(mnn/lib)

# Face detection on live camera
add_executable(face_det_live src/face_det_live_main.cpp src/UltraFace.cpp src/LibCamera.cpp)
target_link_libraries(face_det_live MNN ${OpenCV_LIBS} "${LIBCAMERA_LIBRARIES}")

# Single frame runner
add_executable(single_inference src/single_inference_main.cpp src/UltraFace.cpp)
target_link_libraries(single_inference MNN ${OpenCV_LIBS})


# Camera stream (no detection)
add_executable(live_camera_stream src/live_camera_stream_main.cpp src/LibCamera.cpp)
target_link_libraries(live_camera_stream ${OpenCV_LIBS} "${LIBCAMERA_LIBRARIES}")