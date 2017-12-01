cmake_minimum_required(VERSION 3.5)
project(vision-only)

#### Compile properties ####
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
add_definitions(-DBOOST_ALL_DYN_LINK)

if (CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DDEBUG)
endif ()

if (CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    add_definitions(-DDEBUG)
endif()

SET(CMAKE_CXX_STANDARD 17)

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
find_package(Boost 1.54 COMPONENTS system log_setup log date_time filesystem thread program_options regex REQUIRED)
find_package(OpenCV 3.0 REQUIRED)
find_package(LibXml2)

include_directories( ${EIGEN3_INCLUDE_DIR}) #Eigen doesn't work either way

#### Sources ####
# All sources of the project. Insert one file per line.
# Don't forget include header files
set(SRC
        src/main.cpp
        include/math/point_t.h
        include/math/vector3d_t.h
        src/math/matrix_t.cpp
        src/math/point_t.cpp
        src/math/vector3d_t.cpp
        src/game_controller/udp_comm_t.cpp
        src/motion/pose2d_t.cpp
        include/motion/pose2d_t.h
        src/motion/odometry_collector_t.cpp
        include/motion/odometry_collector_t.h
        include/math/angle_tools.h
        include/math/eigen.h
        src/math/angle_tools.cpp
        include/config/property_reader_t.h
        include/log/console.h
        src/log/coloured_sink_t.cpp
        include/log/coloured_sink_t.h
        src/log/trivial_logger_t.cpp
        include/log/trivial_logger_t.h
        src/vision_application_t.cpp
        include/vision_application_t.h
        src/color_t.cpp
        include/color_t.h
        src/tool/accumulator_t.cpp
        include/tool/accumulator_t.h
        include/tool/rate_t.h
#        include/vision/detectors/AngleDetector.h
        include/vision/detectors/field_detector_t.h
        include/vision/detectors/line_detector_t.h
#        include/vision/Vision.h
        include/vision/vision_utils.h
#        src/vision/detectors/AngleDetector.cpp
        src/vision/detectors/field_detector_t.cpp
        src/vision/detectors/line_detector_t.cpp
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
        src/config/strategies/config_path_arguments_parsing_strategy_t.cpp
        src/config/strategies/configuration_strategy_t.cpp
        src/config/strategies/debug_mode_arguments_parsing_strategy_t.cpp
        src/config/strategies/help_arguments_parsing_strategy_t.cpp
        src/config/arguments_parser_t.cpp
        src/config/command_arguments_t.cpp
        src/config/configuration_parser_t.cpp
        src/config/configuration_file_loader_t.cpp
        src/config/single_option_arguments_parsing_strategy_t.cpp
        src/config/property_reader_t.cpp
        include/config/strategies/config_path_arguments_parsing_strategy_t.h
        include/config/strategies/debug_mode_arguments_parsing_strategy_t.h
        include/config/strategies/help_arguments_parsing_strategy_t.h
        include/config/arguments_parser_t.h
        include/config/arguments_parsing_strategy_t.h
        include/config/command_arguments_t.h
        include/config/configuration_parser_t.h
        include/config/configuration_strategy_t.h
        include/config/configuration_file_loader_t.h
        include/config/single_option_arguments_parsing_strategy_t.h
        include/vision/vision_t.h
        src/vision/vision_t.cpp
        include/vision/vision_processor_t.h
        src/vision/white_ball_vision_processor_t.cpp
        include/vision/white_ball_vision_processor_t.h
        include/config/strategies/white_ball_vision_processor_configuration_strategy_t.h
        src/config/strategies/white_ball_vision_processor_configuration_strategy_t.cpp
        src/config/strategies/white_ball_vision_processor_arguments_parsing_strategy_t.cpp
        include/config/strategies/white_ball_vision_processor_arguments_parsing_strategy_t.h
        include/tool/time_based_holder_t.h)

#### Build targets #####
add_executable(vision_only ${SRC})

target_link_libraries(vision_only
        ${CMAKE_THREAD_LIBS_INIT}
        ${JPEG_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${LIBXML2_LIBRARIES})

target_include_directories(vision_only PUBLIC
        ${JPEG_INCLUDE_DIR}
        ${Boost_INCLUDE_DIR}
        ${JPEG_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${LIBXML2_INCLUDE_DIR})
