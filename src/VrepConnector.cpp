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


VrepConnector::VrepConnector() {
    std::string server_ip("127.0.0.1");
    int server_port = 19997;
    m_client_id = simxStart((simxChar *) server_ip.c_str(), server_port, true, true, 5000, 5);

    if (m_client_id == -1) {
        std::cout << "Can't connect with sim" << std::endl;
    }
    simxSynchronous(m_client_id, 1);
    simxStartSimulation(m_client_id, simx_opmode_oneshot);
}

VrepConnector::VrepConnector(std::string server_ip, int server_port) {
    m_client_id = simxStart((simxChar *) server_ip.c_str(), server_port, true, true, 5000, 5);
    if (m_client_id == -1) {
        std::cout << "Can't connect with sim" << std::endl;
    }
    simxSynchronous(m_client_id, 1);
    simxStartSimulation(m_client_id, simx_opmode_oneshot);
}

int VrepConnector::GetClientID() {
    return m_client_id;
}

void VrepConnector::Disconnect() {
    simxStopSimulation(m_client_id, simx_opmode_oneshot_wait);
    simxFinish(m_client_id);
}