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


#include <string>
#include <unordered_map>
#include <boost/core/noncopyable.hpp>
#include <cassert>

class ServiceLocator : boost::noncopyable {
public:
    ServiceLocator() = default;

    template <class Service>
    void AddService(std::string name, Service* service) {
        m_ServiceHash.emplace(std::move(name), service);
    }

    void RemoveService(const std::string& name) {
        auto iter = m_ServiceHash.find(name);
        if (iter != m_ServiceHash.end()) {
            m_ServiceHash.erase(iter);
        }
    }

    template <class Service>
    Service* GetService(const std::string& name) {
        auto iter = m_ServiceHash.find(name);
        assert(iter != m_ServiceHash.end());
        return (Service*) iter->second;
    }

private:
    std::unordered_map<std::string, void*> m_ServiceHash;

};


