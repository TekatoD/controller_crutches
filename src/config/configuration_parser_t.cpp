/*!
 *  \autor arssivka
 *  \date 10/24/17
 */

#include "config/configuration_parser_t.h"
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string.hpp>

using namespace drwn;

boost::property_tree::ptree configuration_parser_t::read_from_file(const std::string& path) {
    auto format = get_file_format(path);
    std::ifstream stream(path);
    return read_from_stream(stream, format);
}

boost::property_tree::ptree configuration_parser_t::read_from_stream(std::istream& stream, config_format format) {
    boost::property_tree::ptree prop;

    switch (format) {
        case config_format::INI: boost::property_tree::read_ini(stream, prop); break;
        case config_format::XML: boost::property_tree::read_xml(stream, prop); break;
        case config_format::JSON: boost::property_tree::read_xml(stream, prop); break;
    }

    return prop;
}

void configuration_parser_t::write_to_file(const std::string& path,
                                           const boost::property_tree::ptree& prop) {
    auto format = get_file_format(path);
    std::ofstream stream(path);
    write_to_stream(stream, format, prop);
}

void configuration_parser_t::write_to_stream(std::ostream& stream,
                                             config_format format,
                                             const boost::property_tree::ptree& prop) {
    switch (format) {
        case config_format::INI: boost::property_tree::write_ini(stream, prop); break;
        case config_format::XML: boost::property_tree::write_xml(stream, prop); break;
        case config_format::JSON: boost::property_tree::write_json(stream, prop); break;
    }
}

config_format configuration_parser_t::get_file_format(const std::string& path) {
    if (boost::ends_with(path, ".ini")) {
        return config_format::INI;
    }

    if (boost::ends_with(path, ".json")) {
        return config_format::JSON;
    }

    if (boost::ends_with(path, ".xml")) {
        return config_format::XML;
    }

    throw std::runtime_error("Unknown file format: " + path);
}
