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
            src/hw/vrep_CM730_t.cpp
            include/hw/vrep_image_source_t.h
            src/hw/vrep_image_source_t.cpp)
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
        include/ball_follower_t.h
        #        include/ball_tracker_t.h
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
        include/state_machine_t.h
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
        src/ball_follower_t.cpp
        src/ball_tracker_t.cpp
        src/hw/linux_motion_timer_t.cpp
        src/main.cpp
        src/state_machine_t.cpp
        include/gamecontroller/robo_cup_game_control_data_t.h
        include/gamecontroller/spl_coach_message.h
        include/gamecontroller/spl_standard_message.h
        src/gamecontroller/game_controller_t.cpp
        include/gamecontroller/game_controller_t.h
        include/udp_comm_t.h
        src/udp_comm_t.cpp
        src/pose_2D_t.cpp
        include/pose_2D_t.h
        src/odometry_collector_t.cpp
        include/odometry_collector_t.h
        src/go_to_t.cpp
        include/go_to_t.h
        include/soccer_behavior_t.h
        src/soccer_behavior_t.cpp
        src/field_t.cpp
        include/field_t.h
        src/ball_searcher_t.cpp
        include/ball_searcher_t.h
        include/hw/CM730_t.h
        src/hw/CM730_t.cpp
        include/math/angle_tools.h
        include/math/eigen.h
        src/math/angle_tools.cpp
        src/property_reader_t.cpp
        include/property_reader_t.h
        src/motion/modules/kicking_t.cpp
        include/motion/modules/kicking_t.h
        include/log/console.h
        src/log/coloured_sink_t.cpp
        include/log/coloured_sink_t.h
        src/log/trivial_logger_t.cpp
        include/log/trivial_logger_t.h
        src/robot_application_t.cpp
        include/robot_application_t.h
        include/behavior/behavior_t.h
        src/behavior/base_behavior_t.cpp
        include/behavior/base_behavior_t.h
        src/hw/LEDs_t.cpp
        include/hw/LEDs_t.h
        src/color_t.cpp
        include/color_t.h
        src/hw/buttons_t.cpp
        include/hw/buttons_t.h
        src/tool/accumulator_t.cpp
        include/tool/accumulator_t.h
        include/tool/rate_t.h
#        include/vision/detectors/AngleDetector.h
#        include/vision/detectors/ball_detector_t.h
        include/vision/detectors/field_detector_t.h
        include/vision/detectors/line_detector_t.h
#        include/vision/Vision.h
        include/vision/vision_utils.h
#        src/vision/detectors/AngleDetector.cpp
#        src/vision/detectors/ball_detector_t.cpp
        src/vision/detectors/field_detector_t.cpp
        src/vision/detectors/line_detector_t.cpp
#        src/vision/Vision.cpp
        include/vision/detectors/preprocessor_t.h
        src/vision/detectors/old_line_preprocessor_t.cpp
        include/vision/detectors/old_line_preprocessor_t.h
        src/vision/detectors/new_line_preprocessor_t.cpp
        include/vision/detectors/new_line_preprocessor_t.h
        src/vision/detectors/field_preprocessor_t.cpp
        include/vision/detectors/field_preprocessor_t.h
        src/vision/detectors/white_ball_detector_t.cpp
        include/vision/detectors/white_ball_detector_t.h
        src/vision/detectors/coloured_ball_detector_t.cpp
        include/vision/detectors/coloured_ball_detector_t.h
        src/vision/detectors/ball_preprocessor_t.cpp
        include/vision/detectors/ball_preprocessor_t.h
        src/config/strategies/ball_searcher_configuration_strategy_t.cpp
        src/config/strategies/ball_tracker_configuration_strategy_t.cpp
        src/config/strategies/config_path_arguments_parsing_strategy_t.cpp
        src/config/strategies/configuration_strategy_t.cpp
        src/config/strategies/debug_mode_arguments_parsing_strategy_t.cpp
        src/config/strategies/game_controller_configuration_strategy_t.cpp
        src/config/strategies/head_configuration_strategy_t.cpp
        src/config/strategies/help_arguments_parsing_strategy_t.cpp
        src/config/strategies/motion_manager_configuration_strategy_t.cpp
        src/config/strategies/walking_configuration_strategy_t.cpp
        src/config/action_configuration_file_loader_t.cpp
        src/config/arguments_parser_t.cpp
        src/config/command_arguments_t.cpp
        src/config/configuration_parser_t.cpp
        src/config/configuration_file_loader_t.cpp
        src/config/single_option_arguments_parsing_strategy_t.cpp
        include/config/strategies/ball_searcher_configuration_strategy_t.h
        include/config/strategies/ball_tracker_configuration_strategy_t.h
        include/config/strategies/config_path_arguments_parsing_strategy_t.h
        include/config/strategies/debug_mode_arguments_parsing_strategy_t.h
        include/config/strategies/game_controller_configuration_strategy_t.h
        include/config/strategies/head_configuration_strategy_t.h
        include/config/strategies/help_arguments_parsing_strategy_t.h
        include/config/strategies/motion_manager_configuration_strategy_t.h
        include/config/strategies/walking_configuration_strategy_t.h
        include/config/action_configuration_file_loader_t.h
        include/config/arguments_parser_t.h
        include/config/arguments_parsing_strategy_t.h
        include/config/command_arguments_t.h
        include/config/configuration_parser_t.h
        include/config/configuration_strategy_t.h
        include/config/configuration_file_loader_t.h
        include/config/single_option_arguments_parsing_strategy_t.h
        src/hw/camera_t.cpp
        include/hw/camera_t.h
        include/hw/image_source_t.h
        include/hw/image_source_failure.h
        src/hw/image_source_failure.cpp
        include/vision/vision_t.h
        include/vision/vision_processor_t.h
        src/vision/white_ball_vision_processor_t.cpp
        include/vision/white_ball_vision_processor_t.h)

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
