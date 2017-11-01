cmake_minimum_required(VERSION 3.5)
project(darwin-soccer)

#### Project options ####
option(CROSSCOMPILE "Compile for real Darwin-OP (V-REP build by default)" OFF)

#### Compile properties ####
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
add_definitions(-DBOOST_ALL_DYN_LINK)

if (CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DDEBUG)
endif ()

if (CROSSCOMPILE)
    add_definitions(-DCROSSCOMPILE) # This definition is used during the initialization in the main
    set(CROSSCOMPILE_FLAGS "-m32 -march=atom -mtune=atom -mfpmath=sse -ffast-math") #-flto was disabled
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CROSSCOMPILE_FLAGS} -static-libstdc++")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CROSSCOMPILE_FLAGS} -static-libstdc++ -std=c++17")

    add_library(hw STATIC
            include/hw/robot_CM730_t.h
            src/hw/robot_CM730_t.cpp
            include/hw/linux_CM730_t.h
            src/hw/linux_CM730_t.cpp)
else (CROSSCOMPILE)
    set(CMAKE_CXX_STANDARD 17)
    find_package(VREP REQUIRED)
    add_library(hw STATIC
            include/hw/vrep_CM730_t.h
            include/hw/vrep_connector_t.h
            src/hw/vrep_connector_t.cpp
            src/hw/vrep_CM730_t.cpp)
    target_link_libraries(hw ${VREP_LIBRARIES})
    target_include_directories(hw PUBLIC ${VREP_REMOTE} ${VREP_INCLUDE})
endif (CROSSCOMPILE)

# Disable some warnings
if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations -Wno-register -Wno-unused-local-typedefs")
endif(CMAKE_COMPILER_IS_GNUCXX)

include_directories(include/)

#### External libraries and packages ####
find_package(Threads REQUIRED)
find_package(JPEG REQUIRED) # This lib is used by the linux camera
find_package(Eigen3 REQUIRED)
# Path to Boost and OpenCV needs to be set manually when crosscompiling
find_package(Boost 1.54 COMPONENTS system log_setup log date_time filesystem thread program_options REQUIRED)
find_package(OpenCV 3.3 REQUIRED)
find_package(LibXml2)

include_directories( ${EIGEN3_INCLUDE_DIR}) #Eigen doesn't work either way

#### Sources ####
# All sources of the project. Insert one file per line.
# Don't forget include header files
set(SRC
        include/motion/modules/action_t.h
        include/BallFollower.h
        #        include/BallTracker.h
        include/Camera.h
        include/hw/FSR_t.h
        include/motion/modules/head_t.h
        include/motion/joint_data_t.h
        include/motion/kinematics_t.h
        include/hw/linux_motion_timer_t.h
        include/math/matrix_t.h
        include/motion/motion_manager_t.h
        include/motion/motion_module_t.h
        include/motion/motion_status_t.h
        include/hw/MX28_t.h
        include/math/point_t.h
        include/StateMachine.h
        include/math/vector_3D_t.h
        include/motion/modules/walking_t.h
        src/math/matrix_t.cpp
        src/math/point_t.cpp
        src/math/vector_3D_t.cpp
        src/motion/modules/walking_t.cpp
        src/motion/modules/head_t.cpp
        src/motion/modules/action_t.cpp
        src/motion/joint_data_t.cpp
        src/motion/kinematics_t.cpp
        src/motion/motion_manager_t.cpp
        src/motion/motion_status_t.cpp
        src/BallFollower.cpp
        src/BallTracker.cpp
        src/hw/linux_motion_timer_t.cpp
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
        include/hw/CM730_t.h
        src/hw/CM730_t.cpp
        include/math/angle_tools.h
        include/math/eigen.h
        src/math/angle_tools.cpp
        src/PropertyReader.cpp
        include/PropertyReader.h
        src/motion/modules/kicking_t.cpp
        include/motion/modules/kicking_t.h
        include/log/console.h
        src/log/сoloured_sink_t.cpp
        include/log/сoloured_sink_t.h
        src/log/trivial_logger_t.cpp
        include/log/trivial_logger_t.h
        src/robot_application_t.cpp
        include/robot_application_t.h
        include/behavior/Behavior.h
        src/behavior/BaseBehavior.cpp
        include/behavior/BaseBehavior.h
        src/hw/LEDs_t.cpp
        include/hw/LEDs_t.h
        src/Color.cpp
        include/Color.h
        src/hw/buttons_t.cpp
        include/hw/buttons_t.h
        src/tool/Accumulator.cpp
        include/tool/Accumulator.h
        include/tool/Rate.h
        include/config/ConfigurationStrategy.h
        src/config/strategies/WalkingConfigurationStrategy.cpp
        include/config/strategies/WalkingConfigurationStrategy.h
        src/config/ConfigurationFileLoader.cpp
        include/config/ConfigurationFileLoader.h
        src/config/ConfigurationParser.cpp
        include/config/ConfigurationParser.h
        include/config/strategies/MotionManagerConfigurationStrategy.h
        src/config/strategies/MotionManagerConfigurationStrategy.cpp
        include/config/ConfigurationParser.h
        src/config/CommandArguments.cpp
        include/config/CommandArguments.h
        src/config/ActionConfigurationFileLoader.cpp
        include/config/ActionConfigurationFileLoader.h
        include/config/CommandArguments.h
        include/BallTracker.h
        include/config/strategies/BallSearcherConfigurationStrategy.h
        include/config/strategies/BallTrackerConfigurationStrategy.h
        include/config/strategies/HeadConfigurationStrategy.h
        include/config/strategies/GameControllerConfigurationStrategy.h
        src/config/strategies/BallSearcherConfigurationStrategy.cpp
        src/config/strategies/BallTrackerConfigurationStrategy.cpp
        src/config/strategies/HeadConfigurationStrategy.cpp
        src/config/strategies/GameControllerConfigurationStrategy.cpp
        src/config/strategies/ConfigurationStrategy.cpp
        src/config/ArgumentsParser.cpp
        include/config/ArgumentsParser.h
        include/config/SingleOptionArgumentsParsingStrategy.h
        include/config/strategies/DebugModeArgumentsParsingStrategy.h
        src/config/strategies/DebugModeArgumentsParsingStrategy.cpp
        src/config/strategies/ConfigPathArgumentsParsingStrategy.cpp
        include/config/strategies/ConfigPathArgumentsParsingStrategy.h
        src/config/SingleOptionArgumentsParsingStrategy.cpp
        include/config/ArgumentsParsingStrategy.h
        src/config/strategies/HelpArgumentsParsingStrategy.cpp
        include/config/strategies/HelpArgumentsParsingStrategy.h
        include/vision/detectors/AngleDetector.h
        include/vision/detectors/BallDetector.h
        include/vision/detectors/BaseDetector.h
        include/vision/detectors/FieldDetector.h
        include/vision/detectors/LineDetector.h
        include/vision/Vision.h
        include/vision/VisionUtils.h
        src/vision/detectors/AngleDetector.cpp
        src/vision/detectors/BallDetector.cpp
        src/vision/detectors/BaseDetector.cpp
        src/vision/detectors/FieldDetector.cpp
        src/vision/detectors/LineDetector.cpp
        src/vision/Vision.cpp
        include/config/strategies/BallDetectorConfigurationStrategy.h src/config/strategies/BallDetectorConfigurationStrategy.cpp)

#### Build targets #####
add_executable(controller ${SRC})

target_link_libraries(controller
        hw
        ${CMAKE_THREAD_LIBS_INIT}
        ${JPEG_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${LIBXML2_LIBRARIES})

target_include_directories(controller PUBLIC
        ${JPEG_INCLUDE_DIR}
        ${Boost_INCLUDE_DIR}
        ${JPEG_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${LIBXML2_INCLUDE_DIR})
