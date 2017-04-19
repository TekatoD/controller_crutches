cmake_minimum_required(VERSION 3.0)
project(darwin-soccer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Threads REQUIRED)
find_package(JPEG REQUIRED)

include_directories(include/)

set(SRC
        include/streamer/httpd.h
        include/streamer/jpeg_utils.h
        include/streamer/mjpg_streamer.h
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
        include/LinuxActionScript.h
        include/LinuxCamera.h
        include/LinuxCM730.h
        include/LinuxDARwIn.h
        include/LinuxMotionTimer.h
        include/LinuxNetwork.h
        include/Matrix.h
        include/minIni.h
        include/mjpg_streamer.h
        include/MotionManager.h
        include/MotionModule.h
        include/MotionStatus.h
        include/MX28.h
        include/Plane.h
        include/Point.h
        include/roboplus.h
        include/StatusCheck.h
        include/Vector.h
        include/VisionMode.h
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
        src/streamer/httpd.cpp
        src/streamer/jpeg_utils.cpp
        src/streamer/mjpg_streamer.cpp
        src/vision/BallFollower.cpp
        src/vision/BallTracker.cpp
        src/vision/Camera.cpp
        src/vision/ColorFinder.cpp
        src/vision/Image.cpp
        src/vision/ImgProcess.cpp
        src/CM730.cpp
        src/LinuxActionScript.cpp
        src/LinuxCamera.cpp
        src/LinuxCM730.cpp
        src/LinuxMotionTimer.cpp
        src/LinuxNetwork.cpp
        src/main.cpp
        src/MX28.cpp
        src/roboplus.cpp
        src/StatusCheck.cpp
        src/VisionMode.cpp)

add_executable(soccer ${SRC})
target_link_libraries(soccer
        ${CMAKE_SOURCE_DIR}/lib/darwin.a
        ${CMAKE_THREAD_LIBS_INIT}
        ${JPEG_LIBRARIES}
        dl)
target_include_directories(soccer PUBLIC
        ${JPEG_INCLUDE_DIR})