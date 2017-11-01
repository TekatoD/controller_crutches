/// \autor arssivka
/// \date 10/31/17

#pragma once


#include <boost/asio.hpp>

namespace drwn {
    class connection_manager_t {
    public:


    private:
        boost::asio::ip::udp::socket m_srv_socket;
    };
}



