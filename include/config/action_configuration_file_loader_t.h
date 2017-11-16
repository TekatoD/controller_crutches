/*!
 *  \autor arssivka
 *  \date 10/26/17
 */
#pragma once


#include <string>

namespace drwn {
    class action_configuration_file_loader_t {
    public:
        static constexpr char DEFAULT_PATH[] = "res/motion_4096.bin";

        action_configuration_file_loader_t() = default;

        void read_motion_file();

        const std::string& get_path() const;

        void set_path(const std::string& path);

    private:
        std::string m_path{DEFAULT_PATH};

    };
}



