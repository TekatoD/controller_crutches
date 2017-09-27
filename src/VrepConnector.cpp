/**
 *  @autor tekatod
 *  @date 9/21/17
 */

#include <VrepConnector.h>
#include <iostream>


#define MAX_EXT_API_CONNECTIONS 255
#define NON_MATLAB_PARSING
extern "C" {
#include "extApi.h"
#include "extApi.c"
#include "extApiPlatform.h"
#include "extApiPlatform.c"
}


VrepConnector::VrepConnector() : m_server_ip("127.0.0.1"), m_port(19997), m_client_id(-1) { }

VrepConnector::VrepConnector(std::string server_ip, int server_port) : m_server_ip(server_ip), m_port(server_port),
                                                                       m_client_id(-1) { }

void VrepConnector::Connect() {
    if(m_client_id == -1) {
        m_client_id = simxStart((simxChar *) m_server_ip.c_str(), m_port, true, true, 5000, 5);
        if (m_client_id == -1) {
            throw std::runtime_error("Can't connect with sim");
        }
        simxSynchronous(m_client_id, 1);
        simxStartSimulation(m_client_id, simx_opmode_oneshot);
    }
}

int VrepConnector::GetClientID() {
    return m_client_id;
}

void VrepConnector::Disconnect() {
    if(m_client_id != -1) {
        simxStopSimulation(m_client_id, simx_opmode_oneshot_wait);
        simxFinish(m_client_id);
        m_client_id = -1;
    }
}

const std::string& VrepConnector::GetServerIp() const {
    return m_server_ip;
}

void VrepConnector::SetServerIp(const std::string& server_ip) {
    m_server_ip = server_ip;
}

int VrepConnector::GetPort() const {
    return m_port;
}

void VrepConnector::SetPort(int port) {
    m_port = port;
}

VrepConnector::~VrepConnector() {
    this->Disconnect();
}
