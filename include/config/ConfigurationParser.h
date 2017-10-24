/*!
 *  \autor arssivka
 *  \date 10/24/17
 */
#pragma once


#include <boost/property_tree/ptree.hpp>
#include <string>

namespace Robot {
    enum class ConfigFormat {
        INI,
        XML,
        JSON
    };

    class ConfigurationParser {
    public:
        static boost::property_tree::ptree ReadFromStream(std::istream& stream, ConfigFormat format);

        static boost::property_tree::ptree ReadFromFile(const std::string& path);

        static ConfigFormat GetFileFormat(const std::string& path);

    };
}



