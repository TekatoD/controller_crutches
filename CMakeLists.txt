cmake_minimum_required(VERSION 3.0)
project(darwin-soccer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(Boost_USE_STATIC_LIBS ON)

find_package(Boost REQUIRED)
find_package(Threads REQUIRED)
find_package(JPEG REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include/)

set(SRC
        include/Action.h
        include/BallFollower.h
        include/BallTracker.h
        include/Camera.h
        include/CM730.h
        include/ColorFinder.h
        include/DARwIn.h
        include/FSR.h
        include/Head.h
        include/Image.h
        include/ImgProcess.h
        include/JointData.h
        include/Kinematics.h
        include/LinuxCamera.h
        include/LinuxCM730.h
        include/LinuxDARwIn.h
        include/LinuxMotionTimer.h
        include/Matrix.h
        include/minIni.h
        include/mjpg_streamer.h
        include/MotionManager.h
        include/MotionModule.h
        include/MotionStatus.h
        include/MX28.h
        include/Plane.h
        include/Point.h
        include/StatusCheck.h
        include/Vector.h
        include/Walking.h
        src/math/Matrix.cpp
        src/math/Plane.cpp
        src/math/Point.cpp
        src/math/Vector.cpp
        include/minIni/minGlue.h
        src/minIni/minIni.c
        include/minIni/minIni.h
        include/minIni/wxMinIni.h
        src/motion/modules
        src/motion/JointData.cpp
        src/motion/Kinematics.cpp
        src/motion/MotionManager.cpp
        src/motion/MotionStatus.cpp
        src/vision/BallFollower.cpp
        src/vision/BallTracker.cpp
        src/vision/Camera.cpp
        src/vision/ColorFinder.cpp
        src/vision/Image.cpp
        src/vision/ImgProcess.cpp
        src/CM730.cpp
        src/LinuxCamera.cpp
        src/LinuxCM730.cpp
        src/LinuxMotionTimer.cpp
        src/main.cpp
        src/MX28.cpp
        src/StatusCheck.cpp
        src/detectors/AngleDetector.cpp
        src/detectors/BallDetector.cpp
        src/detectors/BaseDetector.cpp
        src/detectors/FieldDetector.cpp
        src/detectors/LineDetector.cpp
        src/BaseModule.cpp
        src/Vision.cpp
        src/VisionUtils.cpp
        include/BaseModule.h
        include/Vision.h
        include/VisionUtils.h
        include/detectors/AngleDetector.h
        include/detectors/BallDetector.h
        include/detectors/BaseDetector.h
        include/detectors/FieldDetector.h
        include/detectors/LineDetector.h
        )

add_executable(soccer ${SRC})

target_link_libraries(soccer
        ${CMAKE_SOURCE_DIR}/lib/darwin.a
        ${CMAKE_THREAD_LIBS_INIT}
        ${JPEG_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        dl)
target_include_directories(soccer PUBLIC
        ${JPEG_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS})