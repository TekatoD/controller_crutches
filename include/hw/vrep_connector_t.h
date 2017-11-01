/**
 *  @autor tekatod
 *  @date 9/21/17
 */
#pragma once

#include <string>

class vrep_connector_t {
public:

    vrep_connector_t();

    vrep_connector_t(std::string server_ip, int server_port);

    const std::string& get_server_ip() const;

    void set_server_ip(const std::string& server_ip);

    void connect();

    int get_client_id();

    void disconnect();

    int get_port() const;

    void set_port(int port);

    ~vrep_connector_t();

private:

    int m_client_id;
    std::string m_server_ip;
    int m_port;

};