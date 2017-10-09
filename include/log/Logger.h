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


#include <boost/log/trivial.hpp>
#include <boost/log/sources/global_logger_storage.hpp>

#define SEVERITY_THRESHOLD logging::trivial::warning
#define LOG(severity) BOOST_LOG_SEV(GlobalLogger::get(),boost::log::trivial::severity)
#define LOG_TRACE   LOG(trace)
#define LOG_DEBUG   LOG(debug)
#define LOG_INFO    LOG(info)
#define LOG_WARNING LOG(warning)
#define LOG_ERROR   LOG(error)
#define LOG_FATAL   LOG(fatal)

namespace Robot {
    using TrivialLogger = boost::log::sources::severity_logger_mt<boost::log::trivial::severity_level>;

    BOOST_LOG_GLOBAL_LOGGER(GlobalLogger, TrivialLogger)
}


