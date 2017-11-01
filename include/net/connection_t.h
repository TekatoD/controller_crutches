/// \autor arssivka
/// \date 10/31/17

#pragma once


#include <boost/asio.hpp>
#include <chrono>

namespace drwn {
    class connection_t {
    public:
        using timestamp_t = std::chrono::steady_clock::time_point;


    private:
        boost::asio::ip::udp::endpoint m_endpoint;
        timestamp_t m_last_packet_received_timestamp;
    };
}



