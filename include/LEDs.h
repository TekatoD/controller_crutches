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
 *  @date 10/17/17
 */

#pragma once


#include <hw/CM730.h>
#include "Color.h"

namespace Robot {
    class LEDs {
    public:
        static LEDs* GetInstance();

        void SetPanelLed(bool first, bool second, bool third);

        void SetHeadLed(const Color& color);

        void SetEyeLed(const Color& color);

        bool IsDebugEnabled() const noexcept;

        void EnableDebug(bool debug) noexcept;

        void Initialize(CM730* cm730) noexcept;

    private:
        LEDs() = default;

    private:
        bool m_debug{true};

        CM730* m_cm730{nullptr};

    };
}


