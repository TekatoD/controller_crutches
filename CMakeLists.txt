cmake_minimum_required(VERSION 3.0)
project(darwin-soccer)

#### Project options ####
option(CROSSCOMPILE "Compile for real Darwin-OP (V-REP build by default)" OFF)

#### Compile properties ####
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(Boost_USE_STATIC_LIBS ON)

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
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

#### Sources ####
# All sources of the project. Insert one file per line.
# Don't forget include header files
set(SRC
        include/motion/modules/Action.h
        include/BallFollower.h
        include/BallTracker.h
        include/Camera.h
        include/VrepCM730.h
        include/ColorFinder.h
        include/FSR.h
        include/motion/modules/Head.h
        include/Image.h
        include/ImgProcess.h
        include/motion/JointData.h
        include/motion/Kinematics.h
        include/LinuxCamera.h
        include/LinuxCM730.h
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
        src/VrepCM730.cpp
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
        include/RobotCM730.h
        src/RobotCM730.cpp
        include/CM730.h
        src/CM730.cpp
        include/VrepConnector.h
        src/VrepConnector.cpp
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
        src/VREPCamera.cpp
        include/VREPCamera.h
        src/Vision.cpp
        include/Vision.h
        src/VisionUtils.cpp
        include/VisionUtils.h
        src/detectors/AngleDetector.cpp
        include/detectors/AngleDetector.h
        src/detectors/BallDetector.cpp
        include/detectors/BallDetector.h
        src/detectors/BaseDetector.cpp
        include/detectors/BaseDetector.h
        src/detectors/FieldDetector.cpp
        include/detectors/FieldDetector.h
        src/detectors/LineDetector.cpp
        include/detectors/LineDetector.h
        src/localization/particle_filter_t.cpp
        include/localization/particle_filter_t.h
        src/localization/localization_util_t.cpp
        include/localization/localization_util_t.h)
        
set(VISION_SRC
    include/minIni.h
    src/minIni/minIni.c
    src/math/Point.cpp
    src/vision/Camera.cpp
    src/vision/ColorFinder.cpp
    src/vision/Image.cpp
    src/vision/ImgProcess.cpp
    src/nodes/field_node.cpp
    src/Vision.cpp
    include/Vision.h
    src/VisionUtils.cpp
    include/VisionUtils.h
    src/detectors/AngleDetector.cpp
    include/detectors/AngleDetector.h
    src/detectors/BallDetector.cpp
    include/detectors/BallDetector.h
    src/detectors/BaseDetector.cpp
    include/detectors/BaseDetector.h
    src/detectors/FieldDetector.cpp
    include/detectors/FieldDetector.h
    src/detectors/LineDetector.cpp
    include/detectors/LineDetector.h)
    
set(VREP_VISION_SRC
    include/VREPCamera.h
    src/VREPCamera.cpp
    include/minIni.h
    src/minIni/minIni.c
    src/math/Point.cpp
    src/vision/Camera.cpp
    src/vision/ColorFinder.cpp
    src/vision/Image.cpp
    src/vision/ImgProcess.cpp
    src/nodes/vrep_node.cpp
    src/Vision.cpp
    include/Vision.h
    src/VisionUtils.cpp
    include/VisionUtils.h
    src/detectors/AngleDetector.cpp
    include/detectors/AngleDetector.h
    src/detectors/BallDetector.cpp
    include/detectors/BallDetector.h
    src/detectors/BaseDetector.cpp
    include/detectors/BaseDetector.h
    src/detectors/FieldDetector.cpp
    include/detectors/FieldDetector.h
    src/detectors/LineDetector.cpp
    include/detectors/LineDetector.h)

#### Build targets #####
add_executable(controller ${SRC})

target_link_libraries(controller
        ${CMAKE_THREAD_LIBS_INIT}
        ${JPEG_LIBRARIES}
        ${BOOST_LIBRARIES}
        ${OpenCV_LIBRARIES}
        Eigen3::Eigen
        ${VREP_LIBRARIES})

target_include_directories(controller PUBLIC
        ${JPEG_INCLUDE_DIR}
        ${VREP_REMOTE}
        ${VREP_INCLUDE}
        ${BOOST_INCLUDE_DIR}
        ${JPEG_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIR})

add_executable(vision_test ${VISION_SRC})

target_link_libraries(vision_test
        ${CMAKE_THREAD_LIBS_INIT}
        ${JPEG_LIBRARIES}
        ${BOOST_LIBRARIES}
        ${OpenCV_LIBRARIES}
        Eigen3::Eigen
        ${VREP_LIBRARIES})

target_include_directories(vision_test PUBLIC
        ${JPEG_INCLUDE_DIR}
        ${VREP_REMOTE}
        ${VREP_INCLUDE}
        ${BOOST_INCLUDE_DIR}
        ${JPEG_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIR})
        
add_executable(vrep_vision ${VREP_VISION_SRC})

target_link_libraries(vrep_vision
        ${CMAKE_THREAD_LIBS_INIT}
        ${JPEG_LIBRARIES}
        ${BOOST_LIBRARIES}
        ${OpenCV_LIBRARIES}
        Eigen3::Eigen
        ${VREP_LIBRARIES})

target_include_directories(vrep_vision PUBLIC
        ${JPEG_INCLUDE_DIR}
        ${VREP_REMOTE}
        ${VREP_INCLUDE}
        ${BOOST_INCLUDE_DIR}
        ${JPEG_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIR})

add_executable(localization_util
        src/localization/localization_util_test.cpp
        src/Pose2D.cpp
        src/localization/particle_filter_t.cpp
        src/localization/localization_util_t.cpp
        src/minIni/minIni.c
        src/math/Point.cpp
        )

target_link_libraries(localization_util
        ${CMAKE_THREAD_LIBS_INIT}
        ${JPEG_LIBRARIES}
        ${BOOST_LIBRARIES}
        ${OpenCV_LIBRARIES}
        Eigen3::Eigen)

target_include_directories(localization_util PUBLIC
        ${BOOST_INCLUDE_DIR}
        ${JPEG_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIR})
