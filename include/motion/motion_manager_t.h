#pragma once

#include <list>
#include "motion_status_t.h"
#include "motion_module_t.h"
#include "hw/CM730_t.h"

namespace drwn {
    class motion_manager_t {
    public:
        int Offset[joint_data_t::NUMBER_OF_JOINTS];

        static motion_manager_t* GetInstance() {
            static motion_manager_t instance;
            return &instance;
        }

        bool initialize(CM730_t* cm730);

        bool reinitialize();

        void process();

        void set_enable(bool enable);

        bool get_enable() const { return m_enabled; }

        void add_module(motion_module_t* module);

        void remove_module(motion_module_t* module);

        void reset_gyro_calibration() {
            m_calibration_status = 0;
            m_fb_gyro_center = 512;
            m_rl_gyro_center = 512;
        }

        int get_calibration_status() const { return m_calibration_status; }

        void set_joint_disable(int index);

        bool is_debug_enabled() const;

        void enable_debug(bool debug);

        void set_joint_offset(int id, int offset);

        int get_joint_offset(int id) const;

    private:
        bool m_debug{false};

        std::list<motion_module_t*> m_modules;
        CM730_t* m_CM730{nullptr};
        bool m_process_enable{false};
        bool m_enabled{false};
        int m_fb_gyro_center{512};
        int m_rl_gyro_center{512};
        int m_calibration_status{0};

        bool m_is_running{false};
        bool m_is_thread_running{false};

        motion_manager_t();
    };
}
