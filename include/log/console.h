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


#include <unistd.h>
#include <ostream>

namespace drwn {
    inline std::ostream& nostyle(std::ostream& stream) {
        stream << "\033[00m";
        return stream;
    }


    inline std::ostream& bold(std::ostream& stream) {
        stream << "\033[1m";
        return stream;
    }


    inline std::ostream& dark(std::ostream& stream) {
        stream << "\033[2m";
        return stream;
    }


    inline std::ostream& underline(std::ostream& stream) {
        stream << "\033[4m";
        return stream;
    }


    inline std::ostream& blink(std::ostream& stream) {
        stream << "\033[5m";
        return stream;
    }


    inline std::ostream& reverse(std::ostream& stream) {
        stream << "\033[7m";
        return stream;
    }


    inline std::ostream& concealed(std::ostream& stream) {
        stream << "\033[8m";
        return stream;
    }


    inline std::ostream& grey(std::ostream& stream) {
        stream << "\033[30m";
        return stream;
    }

    inline std::ostream& red(std::ostream& stream) {
        stream << "\033[31m";
        return stream;
    }

    inline std::ostream& green(std::ostream& stream) {
        stream << "\033[32m";
        return stream;
    }

    inline std::ostream& yellow(std::ostream& stream) {
        stream << "\033[33m";
        return stream;
    }

    inline std::ostream& blue(std::ostream& stream) {
        stream << "\033[34m";
        return stream;
    }

    inline std::ostream& magenta(std::ostream& stream) {
        stream << "\033[35m";
        return stream;
    }

    inline std::ostream& cyan(std::ostream& stream) {
        stream << "\033[36m";
        return stream;
    }

    inline std::ostream& white(std::ostream& stream) {
        stream << "\033[37m";
        return stream;
    }


    inline std::ostream& ongrey(std::ostream& stream) {
        stream << "\033[40m";
        return stream;
    }

    inline std::ostream& onred(std::ostream& stream) {
        stream << "\033[41m";
        return stream;
    }

    inline std::ostream& ongreen(std::ostream& stream) {
        stream << "\033[42m";
        return stream;
    }

    inline std::ostream& onyellow(std::ostream& stream) {
        stream << "\033[43m";
        return stream;
    }

    inline std::ostream& onblue(std::ostream& stream) {
        stream << "\033[44m";
        return stream;
    }

    inline std::ostream& onmagenta(std::ostream& stream) {
        stream << "\033[45m";
        return stream;
    }

    inline std::ostream& oncyan(std::ostream& stream) {
        stream << "\033[46m";
        return stream;
    }

    inline std::ostream& onwhite(std::ostream& stream) {
        stream << "\033[47m";
        return stream;
    }
}