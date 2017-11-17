/**
 * Copyright 2016 Arseniy Ivin <arssivka@yandex.ru>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  @autor arssivka
 *  @date 8/16/17
 */

#include "config/property_reader_t.h"
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string.hpp>


config_format property_reader_t::get_format_from_extension(std::string path) {
    boost::to_lower(path);
    if (boost::ends_with(path, ".xml")) return CONFIG_FORMAT_XML;
    if (boost::ends_with(path, ".json")) return CONFIG_FORMAT_JSON;
    if (boost::ends_with(path, ".ini")) return CONFIG_FORMAT_INI;
    return CONFIG_FORMAT_UNKNOWN;
}

bool property_reader_t::from_stream(std::istream& stream,
                                    config_format format,
                                    boost::property_tree::ptree& prop) {
    namespace pt = boost::property_tree;
    try {
        switch (format) {
            case CONFIG_FORMAT_XML:
                pt::read_xml(stream, prop);
                return true;
            case CONFIG_FORMAT_JSON:
                pt::read_json(stream, prop);
                return true;
            case CONFIG_FORMAT_INI:
                pt::read_ini(stream, prop);
                return true;
            default:
                return false;
        }
    } catch (const pt::file_parser_error& e) {
        return false;
    }
}

bool property_reader_t::to_stream(std::ostream& stream,
                                  config_format format,
                                  const boost::property_tree::ptree& prop) {
    namespace pt = boost::property_tree;
    switch (format) {
        case CONFIG_FORMAT_XML:
            pt::write_xml(stream, prop);
            return true;
        case CONFIG_FORMAT_JSON:
            pt::write_json(stream, prop);
            return true;
        case CONFIG_FORMAT_INI:
            pt::write_ini(stream, prop);
            return true;
        default:
            return false;
    }
}

bool property_reader_t::from_file(const std::string& path,
                                  boost::property_tree::ptree& prop) {
    std::ifstream stream(path);
    config_format format = get_format_from_extension(path);
    if (!stream) return false;
    return from_stream(stream, format, prop);
}

bool property_reader_t::to_file(const std::string& path,
                                const boost::property_tree::ptree& prop) {
    std::ofstream stream(path);
    config_format format = get_format_from_extension(path);
    if (!stream) return false;
    return to_stream(stream, format, prop);
}
