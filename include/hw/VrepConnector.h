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

    const std::string& GetServerIp() const;

    void SetServerIp(const std::string& m_server_ip);

    void Connect();

    int GetClientID();

    void Disconnect();

    int GetPort() const;

    void SetPort(int m_port);

    ~VrepConnector();

private:

    int m_client_id;
    std::string m_server_ip;
    int m_port;

};