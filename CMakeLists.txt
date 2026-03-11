cmake_minimum_required(VERSION 3.10)
project(CameraCalibration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)

add_executable(CameraCalibration
    src/Main.cpp
    src/CalibrationCamera.cpp
    src/CannyDevernay.cpp
)

target_include_directories(CameraCalibration
    PRIVATE
        include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(CameraCalibration PRIVATE ${OpenCV_LIBS})
