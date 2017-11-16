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

#pragma once

#include <boost/log/sinks/basic_sink_backend.hpp>


namespace drwn {
    class coloured_sink_t
            : public boost::log::sinks::basic_formatted_sink_backend<char, boost::log::sinks::synchronized_feeding>  {
    public:
        static void consume(const boost::log::record_view& rec, const string_type& formatted_string);

    };
}


