/**
 *  @autor tekatod
 *  @date 11/3/17
 */
#pragma once

#include <stdexcept>

namespace drwn {
    class image_source_failure : public std::runtime_error {
    public:
        image_source_failure(std::string what_happened);
    };
}