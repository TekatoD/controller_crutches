/// \autor arssivka
/// \date 11/1/17

#pragma once


#include <boost/asio.hpp>

namespace drwn {
    class server_listener_t {
    public:
        virtual void process(const boost::asio::ip::udp::endpoint& ep,
                             boost::asio::streambuf& stream) = 0;

        virtual server_listener_t() = default;
    };
}



