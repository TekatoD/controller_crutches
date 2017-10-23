cmake_minimum_required(VERSION 3.0)
project(darwin-soccer)

#### Project options ####
option(CROSSCOMPILE "Compile for real Darwin-OP (V-REP build by default)" OFF)

#### Compile properties ####
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
add_definitions(-DBOOST_ALL_DYN_LINK)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release FORCE)
endif()

if (CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DDEBUG)
endif()

if (CROSSCOMPILE)
    add_definitions(-DCROSSCOMPILE) # This definition is used during the initialization in the main
    set(CROSSCOMPILE_FLAGS "-m32 -march=atom -mtune=atom -mfpmath=sse -flto -ffast-math")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CROSSCOMPILE_FLAGS} -static-libstdc++")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CROSSCOMPILE_FLAGS} -static-libstdc++")
endif (CROSSCOMPILE)

include_directories(include/)

#### External libraries and packages ####
find_package(Threads REQUIRED)
find_package(JPEG REQUIRED) # This lib is used by the linux camera
find_package(VREP REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# Path to Boost and OpenCV needs to be set manually when crosscompiling
find_package(Boost 1.54 COMPONENTS system log date_time filesystem thread REQUIRED)
find_package(OpenCV 3.3 REQUIRED)

#### Sources ####
# All sources of the project. Insert one file per line.
# Don't forget include header files
set(SRC
        include/motion/modules/Action.h
        include/BallFollower.h
#        include/BallTracker.h
        include/Camera.h
        include/hw/VrepCM730.h
        include/hw/FSR.h
        include/motion/modules/Head.h
        include/motion/JointData.h
        include/motion/Kinematics.h
        include/hw/LinuxCM730.h
        include/hw/LinuxMotionTimer.h
        include/math/Matrix.h
        include/minIni.h
        include/motion/MotionManager.h
        include/motion/MotionModule.h
        include/motion/MotionStatus.h
        include/hw/MX28.h
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
        src/hw/VrepCM730.cpp
        src/hw/LinuxCM730.cpp
        src/hw/LinuxMotionTimer.cpp
        src/main.cpp
        src/StateMachine.cpp
        include/gamecontroller/RoboCupGameControlData.h
        include/gamecontroller/SPLCoachMessage.h
        include/gamecontroller/SPLStandardMessage.h
        src/gamecontroller/GameController.cpp
        include/gamecontroller/GameController.h
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
        src/Field.cpp
        include/Field.h
        src/BallSearcher.cpp
        include/BallSearcher.h
        include/hw/RobotCM730.h
        src/hw/RobotCM730.cpp
        include/hw/CM730.h
        src/hw/CM730.cpp
        include/hw/VrepConnector.h
        src/hw/VrepConnector.cpp
        include/math/AngleTools.h
        include/math/Eigen.h
        src/math/AngleTools.cpp
        src/ServiceLocator.cpp
        include/ServiceLocator.h
        src/ServiceInterface.cpp
        include/ServiceInterface.h
        include/Singleton.h
        src/PropertyReader.cpp
        include/PropertyReader.h
        src/motion/modules/Kicking.cpp
        include/motion/modules/Kicking.h
        include/log/Console.h
        src/log/ColouredSink.cpp
        include/log/ColouredSink.h
        src/log/Logger.cpp
        include/log/Logger.h
        src/RobotApplication.cpp
        include/RobotApplication.h
        include/behavior/Behavior.h
        src/behavior/BaseBehavior.cpp
        include/behavior/BaseBehavior.h
        src/hw/LEDs.cpp
        include/hw/LEDs.h
        src/Color.cpp
        include/Color.h
        src/hw/Buttons.cpp
        include/hw/Buttons.h
        src/tool/Accumulator.cpp
        include/tool/Accumulator.h)

#### Build targets #####
add_executable(controller ${SRC})

target_link_libraries(controller
        ${CMAKE_THREAD_LIBS_INIT}
        ${JPEG_LIBRARIES}
        ${Boost_LIBRARIES}
#        ${OpenCV_LIBRARIES}
        Eigen3::Eigen
        ${VREP_LIBRARIES})

target_include_directories(controller PUBLIC
        ${JPEG_INCLUDE_DIR}
        ${VREP_REMOTE}
        ${VREP_INCLUDE}
        ${Boost_INCLUDE_DIR}
        ${JPEG_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIR})
