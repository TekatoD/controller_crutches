/*!
 *  \autor arssivka
 *  \date 10/24/17
 */

#include "config/ConfigurationParser.h"
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string.hpp>

using namespace drwn;

boost::property_tree::ptree ConfigurationParser::ReadFromFile(const std::string& path) {
    auto format = GetFileFormat(path);
    std::ifstream stream(path);
    return ReadFromStream(stream, format);
}

boost::property_tree::ptree ConfigurationParser::ReadFromStream(std::istream& stream, ConfigFormat format) {
    boost::property_tree::ptree prop;

    switch (format) {
        case ConfigFormat::INI: boost::property_tree::read_ini(stream, prop); break;
        case ConfigFormat::XML: boost::property_tree::read_xml(stream, prop); break;
        case ConfigFormat::JSON: boost::property_tree::read_xml(stream, prop); break;
    }

    return prop;
}

void ConfigurationParser::WriteToFile(const std::string& path,
                                      const boost::property_tree::ptree& prop) {
    auto format = GetFileFormat(path);
    std::ofstream stream(path);
    WriteToStream(stream, format, prop);
}

void ConfigurationParser::WriteToStream(std::ostream& stream,
                                        ConfigFormat format,
                                        const boost::property_tree::ptree& prop) {
    switch (format) {
        case ConfigFormat::INI: boost::property_tree::write_ini(stream, prop); break;
        case ConfigFormat::XML: boost::property_tree::write_xml(stream, prop); break;
        case ConfigFormat::JSON: boost::property_tree::write_json(stream, prop); break;
    }
}

ConfigFormat ConfigurationParser::GetFileFormat(const std::string& path) {
    if (boost::ends_with(path, ".ini")) {
        return ConfigFormat::INI;
    }

    if (boost::ends_with(path, ".json")) {
        return ConfigFormat::JSON;
    }

    if (boost::ends_with(path, ".xml")) {
        return ConfigFormat::XML;
    }

    throw std::runtime_error("Unknown file format: " + path);
}
