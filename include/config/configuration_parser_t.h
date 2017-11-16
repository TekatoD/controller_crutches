/*!
 *  \autor arssivka
 *  \date 10/24/17
 */
#pragma once


#include <boost/property_tree/ptree.hpp>
#include <string>

namespace drwn {
    enum class config_format {
        INI,
        XML,
        JSON
    };

    class configuration_parser_t {
    public:
        static boost::property_tree::ptree read_from_file(const std::string& path);

        static boost::property_tree::ptree read_from_stream(std::istream& stream, config_format format);

        static void write_to_file(const std::string& path, const boost::property_tree::ptree& prop);

        static void write_to_stream(std::ostream& stream, config_format format, const boost::property_tree::ptree& prop);

        static config_format get_file_format(const std::string& path);

    };
}



