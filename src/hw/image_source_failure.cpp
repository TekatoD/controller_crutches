/**
 *  @autor tekatod
 *  @date 11/3/17
 */

#include "hw/image_source_failure.h"

drwn::image_source_failure::image_source_failure(std::string what_happened)
        : std::runtime_error(what_happened) { }
