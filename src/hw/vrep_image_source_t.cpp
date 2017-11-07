/**
 *  @autor tekatod
 *  @date 11/3/17
 */

#include <log/trivial_logger_t.h>
#include "hw/vrep_image_source_t.h"
#include "vision/vision_utils.h"
#include "hw/image_source_failure.h"

#define MAX_EXT_API_CONNECTIONS 255
#define NON_MATLAB_PARSING
extern "C" {
#include "extApi.h"
#include "extApiPlatform.h"
}


int drwn::vrep_image_source_t::get_client_id() const {
    return m_client_id;
}

void drwn::vrep_image_source_t::set_client_id(int client_id) {
    if(client_id != -1) {
        m_client_id = client_id;
    }
    else {
        throw std::runtime_error("VREP IMAGE SOURCE: Wrong client id");
    }
}

drwn::vrep_image_source_t::vrep_image_source_t(std::string sensor_name, int width, int height)
        : m_sensor_name(std::move(sensor_name)), m_width{width}, m_height{height} {
}

void drwn::vrep_image_source_t::connect() {
    if(m_client_id != -1) {
        if(simxGetObjectHandle(m_client_id, (simxChar *) m_sensor_name.c_str(),
                               &m_sensor_handle, simx_opmode_oneshot_wait) != simx_return_ok) {
            throw std::runtime_error("VREP IMAGE SOURCE: Can't connect to the vrep camera");
        }
        int sim_res_x;
        int sim_res_y;
        while(simxGetObjectIntParameter(m_client_id, m_sensor_handle, 1002, &sim_res_x, simx_opmode_oneshot) != simx_return_ok) { }
        while(simxGetObjectIntParameter(m_client_id, m_sensor_handle, 1003, &sim_res_y, simx_opmode_oneshot) != simx_return_ok) { }
        LOG_INFO << "VREP IMAGE SOURCE: Set up camera resolution: " << sim_res_x << " X " << sim_res_y;
        if(sim_res_x != m_width || sim_res_y != m_height) {
            throw std::runtime_error("VREP IMAGE SOURCE: Missmatch of specified resolution with vrep camera");
        }

        int code;
        code = simxGetVisionSensorImage(m_client_id,m_sensor_handle, m_resolution,
                                       &m_binary_image, 0, simx_opmode_streaming);
        LOG_INFO << "VREP IMAGE SOURCE: Set up camera streaming: " << code;

    }
}

cv::Mat drwn::vrep_image_source_t::capture_frame() const {
    simxSynchronousTrigger(m_client_id);
    if(simxGetVisionSensorImage(m_client_id, m_sensor_handle, m_resolution,
                                &m_binary_image, 0, simx_opmode_buffer) == simx_return_ok) {
        if(m_resolution[0] != m_width || m_resolution[1] != m_height) {
            LOG_WARNING << "VREP IMAGE SOURCE: Received a broken frame from vrep";
            throw image_source_failure("VREP IMAGE SOURCE: Received a broken frame from vrep");
        }
        else {
            if(m_debug) {
                LOG_DEBUG << "VREP IMAGE SOURCE: Received normal image";
            }
            vision_utils::vertical_flip_rgb(m_binary_image, m_width, m_height);
            vision_utils::rgb_to_bgr(m_binary_image, m_width, m_height);
            return cv::Mat(m_height, m_width, CV_8UC3, m_binary_image);
        }
    }
    else {
        throw image_source_failure("VREP IMAGE SOURCE: Vrep didn't respond in time");
    }
}

bool drwn::vrep_image_source_t::is_debug_enabled() const {
    return m_debug;
}

void drwn::vrep_image_source_t::enable_debug(bool debug) {
    m_debug = debug;
}
