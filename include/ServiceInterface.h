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

class ServiceInterface {
public:
    ServiceInterface() = default;
    ServiceInterface(const ServiceInterface&) = delete;
    ServiceInterface& operator=(const ServiceInterface&) = delete;

    ServiceInterface(ServiceInterface&&) = delete;
    ServiceInterface& operator=(ServiceInterface&&) = delete;

    virtual void ParseArguments(int argc, char** argv) = 0;

    virtual void LoadConfig() = 0;

    virtual void SaveConfig() = 0;

    virtual void OnStart() = 0;

    virtual void OnStop() = 0;

    const std::string& GetConfigFile() const noexcept;

    void SetConfigFile(const std::string& configFile);

    virtual ~ServiceInterface() = default;

private:
    std::string m_ConfigFile;

};


