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
        static boost::property_tree::ptree ReadFromFile(const std::string& path);

        static boost::property_tree::ptree ReadFromStream(std::istream& stream, ConfigFormat format);

        static void WriteToFile(const std::string& path, const boost::property_tree::ptree& prop);

        static void WriteToStream(std::ostream& stream, ConfigFormat format, const boost::property_tree::ptree& prop);

        static ConfigFormat GetFileFormat(const std::string& path);

    };
}



