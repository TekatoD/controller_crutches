cmake_minimum_required(VERSION 3.8)
project(darwin-soccer)
#-DCMAKE_TOOLCHAIN_FILE=/home/tekatod/develop/controller_crutches/toolchain.cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

find_package(Threads REQUIRED)
find_package(JPEG REQUIRED)
find_package(VREP REQUIRED)

include_directories(include/)

include_directories(${VREP_INCLUDE})
include_directories(${VREP_REMOTE})

set(SRC
        include/motion/modules/Action.h
        include/BallFollower.h
        include/BallTracker.h
        include/Camera.h
        include/CM730vrep.h
        include/ColorFinder.h
        include/DARwIn.h
        include/FSR.h
        include/motion/modules/Head.h
        include/Image.h
        include/ImgProcess.h
        include/motion/JointData.h
        include/motion/Kinematics.h
        include/LinuxCamera.h
        include/LinuxCM730.h
        include/LinuxDARwIn.h
        include/LinuxMotionTimer.h
        include/math/Matrix.h
        include/minIni.h
        include/motion/MotionManager.h
        include/MotionModule.h
        include/motion/MotionStatus.h
        include/MX28.h
        include/math/Point.h
        include/StateMachine.h
        include/math/Vector.h
        include/motion/modules/Walking.h
        src/math/Matrix.cpp
        src/math/Point.cpp
        src/math/Vector.cpp
        include/minIni/minGlue.h
        src/minIni/minIni.c
        include/minIni/minIni.h
        include/minIni/wxMinIni.h
        src/motion/modules/Walking.cpp
        src/motion/modules/Head.cpp
        src/motion/modules/Action.cpp
        src/motion/JointData.cpp
        src/motion/Kinematics.cpp
        src/motion/MotionManager.cpp
        src/motion/MotionStatus.cpp
        src/BallFollower.cpp
        src/BallTracker.cpp
        src/vision/Camera.cpp
        src/vision/ColorFinder.cpp
        src/vision/Image.cpp
        src/vision/ImgProcess.cpp
        src/CM730vrep.cpp
        src/LinuxCamera.cpp
        src/LinuxCM730.cpp
        src/LinuxMotionTimer.cpp
        src/main.cpp
        src/MX28.cpp
        src/StateMachine.cpp

        include/gamecontroller/RoboCupGameControlData.h
        include/gamecontroller/SPLCoachMessage.h
        include/gamecontroller/SPLStandardMessage.h
        src/GameController.cpp include/GameController.h

        include/UdpComm.h
        src/UdpComm.cpp
#        src/Speech.cpp
#        include/Speech.h
        src/Pose2D.cpp
        include/Pose2D.h
        src/OdometryCollector.cpp
        include/OdometryCollector.h
        src/GoTo.cpp
        include/GoTo.h
        include/SoccerBehavior.h
        src/SoccerBehavior.cpp
        include/GoalieBehavior.h
        src/GoalieBehavior.cpp
        src/Field.cpp
        include/Field.h
        src/BallSearcher.cpp
        include/BallSearcher.h
        include/CM730robot.h
        src/CM730robot.cpp
        include/CM730.h
        src/CM730.cpp
        include/VrepConnector.h
        src/VrepConnector.cpp)

add_executable(soccer ${SRC})
target_link_libraries(soccer
        ${CMAKE_THREAD_LIBS_INIT}
        ${JPEG_LIBRARIES}
        pthread
        ${VREP_LIBRARIES})
target_include_directories(soccer PUBLIC
        ${JPEG_INCLUDE_DIR})