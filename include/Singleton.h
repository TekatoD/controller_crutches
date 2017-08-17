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


#include <mutex>
#include <memory>
#include <cassert>

template <class T>
class Singleton {
public:
    Singleton() = delete;

    Singleton(const Singleton&) = delete;
    Singleton& operator=(const Singleton& ) = delete;

    Singleton(Singleton&&) = delete;
    Singleton& operator=(Singleton&&) = delete;

    virtual ~Singleton() = default;

    template <class... Args>
    static void Initialize(Args&&... args) {
        std::call_once(m_InitializedFlag, [&args...] {
            m_Instance.reset(new T(std::forward<Args>(args)...));
        });
    }

    static T* GetInstance() {
        assert(m_Instance != nullptr);
        return (T*) m_Instance;
    }

private:
    static std::once_flag m_InitializedFlag;
    static std::unique_ptr<T> m_Instance;

};


