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
 *  @date 10/9/17
 */

#include <iostream>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include "log/сoloured_sink_t.h"
#include "log/console.h"


void drwn::сoloured_sink_t::consume(const boost::log::record_view& rec, const string_type& formatted_string)
{
    auto severity = rec.attribute_values()["Severity"].extract<boost::log::trivial::severity_level>();

    if (severity) {
        // Change text color
        switch (*severity) {
            case boost::log::trivial::trace:
                std::cout << drwn::white;
                break;
            case boost::log::trivial::debug:
                std::cout << drwn::cyan;
                break;
            case boost::log::trivial::info:
                std::cout << drwn::green;
                break;
            case boost::log::trivial::warning:
                std::cout << drwn::yellow;
                break;
            case boost::log::trivial::error:
                std::cout << drwn::red;
                break;
            case boost::log::trivial::fatal:
                std::cout << drwn::red << drwn::bold;
                break;
            default:
                break;
        }
    }

    std::cout << formatted_string;

    if (severity) {
        std::cout << drwn::nostyle;
    }

    std::cout << std::endl;
}