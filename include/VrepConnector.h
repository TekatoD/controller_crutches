/**
 *  @autor tekatod
 *  @date 9/21/17
 */
#pragma once

#include <string>

class VrepConnector {
public:

    VrepConnector();

    VrepConnector(std::string server_ip, int server_port);

    int GetClientID();

    void Disconnect();

private:

    int m_client_id;

};