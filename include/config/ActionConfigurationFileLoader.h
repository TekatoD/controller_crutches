/*!
 *  \autor arssivka
 *  \date 10/26/17
 */
#pragma once


#include <string>

namespace drwn {
    class ActionConfigurationFileLoader {
    public:
        static constexpr char DEFAULT_PATH[] = "res/motion_4096.bin";

        ActionConfigurationFileLoader() = default;

        void ReadMotionFile();

        const std::string& GetPath() const;

        void SetPath(const std::string& path);

    private:
        std::string m_path{DEFAULT_PATH};

    };
}



