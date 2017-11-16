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

#pragma once


#include <istream>
#include <boost/property_tree/ptree.hpp>


enum config_format {
    CONFIG_FORMAT_UNKNOWN,
    CONFIG_FORMAT_XML,
    CONFIG_FORMAT_JSON,
    CONFIG_FORMAT_INI
};

class property_reader_t {
public:
    static config_format get_format_from_extension(std::string path);

    static bool from_stream(std::istream& stream,
                            config_format format,
                            boost::property_tree::ptree& prop);

    static bool to_stream(std::ostream& stream,
                          config_format format,
                          const boost::property_tree::ptree& prop);

    static bool from_file(const std::string& path, boost::property_tree::ptree& prop);

    static bool to_file(const std::string& path, const boost::property_tree::ptree& prop);

};


