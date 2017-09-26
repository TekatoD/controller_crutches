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

    void Connect();

    int GetClientID();

    void Disconnect();

    ~VrepConnector() = default;

private:

    int m_client_id;
    std::string m_server_ip;
    int m_port;

};