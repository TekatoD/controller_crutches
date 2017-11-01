/// \autor arssivka
/// \date 10/31/17

#pragma once


#include <string>
#include <cstdint>

namespace drwn {
    struct packet_header_t {
        int32_t crc;
        std::string data;
    };
}



