cmake_minimum_required(VERSION 3.0)
project(darwin-soccer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(Boost_USE_STATIC_LIBS ON)

option(CROSSCOMPILE "Compile for real Darwin-OP (V-REP build by default)" OFF)

find_package(Threads REQUIRED)
find_package(JPEG REQUIRED) # This lib is used by the linux camera
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# Path to Boost and OpenCV needs to be set manually when crosscompiling
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include/)

if (CROSSCOMPILE)
    add_definitions(-DCROSSCOMPILE) # This definition is used during the initialization in the main
    set(CROSSCOMPILE_FLAGS "-m32 -march=atom -mtune=atom -mfpmath=sse -flto -ffast-math")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CROSSCOMPILE_FLAGS} -static-libstdc++")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CROSSCOMPILE_FLAGS} -static-libstdc++")
endif (CROSSCOMPILE)

set(SRC
        include/motion/modules/Action.h
        include/BallFollower.h
        include/BallTracker.h
        include/Camera.h
        include/CM730.h
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
        src/CM730.cpp
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
        include/math/AngleTools.h
        include/math/Eigen.h
        src/math/AngleTools.cpp src/ServiceLocator.cpp include/ServiceLocator.h src/ServiceInterface.cpp include/ServiceInterface.h include/Singleton.h src/PropertyReader.cpp include/PropertyReader.h)

add_executable(controller ${SRC})

target_link_libraries(controller
        ${CMAKE_THREAD_LIBS_INIT}
        ${JPEG_LIBRARIES}
        ${BOOST_LIBRARIES}
        ${OpenCV_LIBRARIES}
        Eigen3::Eigen)

target_include_directories(controller PUBLIC
        ${BOOST_INCLUDE_DIR}
        ${JPEG_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIR})