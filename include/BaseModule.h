//
// Created by pav on 07/04/2017.
//

#ifndef MODULES_BASEMODULE_H
#define MODULES_BASEMODULE_H


#include <atomic>
#include <string>
//#include <rrc/shared_buffer.h>
//#include <rrc/core.h>
//#include <rrc/async_worker.h>

namespace ant {
    class BaseModule {
    protected:
//        rrc::core &m_core;
//        rrc::async_worker<rrc::shared_buffer> m_buff;
//        rrc::topic_callback m_callback;
        std::string m_prop_file;
        std::string m_topic_listen;
        std::string m_topic;
        std::atomic<bool> m_debug;
        std::atomic<bool> m_enabled;
    public:
        BaseModule(std::string prop_file = std::string()) :
            m_enabled(true), m_prop_file(prop_file) {
            this->update_properties();
        }

        virtual bool enabled() const {
            return m_enabled;
        }

        virtual void set_enabled(bool enabled) {
            if (m_enabled != enabled) {
                if (m_enabled) {
                    this->disable();
                } else {
                    this->enable();
                }
            }
            m_enabled = enabled;
        }

        virtual const std::string &property_file() const {
            return m_prop_file;
        }

        virtual void set_property_file(const std::string &prop_file) {
            if (prop_file != m_prop_file) {
                m_prop_file = prop_file;
                this->update_properties();
            }
        }

        virtual ~BaseModule() {
            this->disable();
        }

    protected:
        virtual void update_properties() {
            if (m_enabled) {
                this->enable();
            } else {
                this->disable();
            }
        }

        virtual void disable() {

        }

        virtual void enable() {

        }
    };

}
#endif //MODULES_BASEMODULE_H
