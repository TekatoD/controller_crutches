/**
 *  @autor tekatod
 *  @date 9/21/17
 */

#include <hw/vrep_connector_t.h>
#include <iostream>


#define MAX_EXT_API_CONNECTIONS 255
#define NON_MATLAB_PARSING
extern "C" {
#include "extApi.h"
#include "extApi.c"
#include "extApiPlatform.h"
#include "extApiPlatform.c"
}


vrep_connector_t::vrep_connector_t()
        : m_server_ip("127.0.0.1"),
          m_port(19997),
          m_client_id(-1) { }

vrep_connector_t::vrep_connector_t(std::string server_ip, int server_port)
        : m_server_ip(std::move(server_ip)),
          m_port(server_port),
          m_client_id(-1) { }

void vrep_connector_t::connect() {
    if(m_client_id == -1) {
        m_client_id = simxStart((simxChar *) m_server_ip.c_str(), m_port, true, true, 5000, 5);
        if (m_client_id == -1) {
            throw std::runtime_error("Can't connect to the sim");
        }
        simxSynchronous(m_client_id, 1);
        simxStartSimulation(m_client_id, simx_opmode_oneshot);
    }
}

int vrep_connector_t::get_client_id() {
    return m_client_id;
}

void vrep_connector_t::disconnect() {
    if(m_client_id != -1) {
        simxStopSimulation(m_client_id, simx_opmode_oneshot_wait);
        simxFinish(m_client_id);
        m_client_id = -1;
    }
}

const std::string& vrep_connector_t::get_server_ip() const {
    return m_server_ip;
}

void vrep_connector_t::set_server_ip(const std::string& server_ip) {
    m_server_ip = server_ip;
}

int vrep_connector_t::get_port() const {
    return m_port;
}

void vrep_connector_t::set_port(int port) {
    m_port = port;
}

vrep_connector_t::~vrep_connector_t() {
    this->disconnect();
}
