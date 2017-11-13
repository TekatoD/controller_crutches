#pragma once


namespace drwn {


    class behavior_t {
    public:
        virtual void process() = 0;

        virtual ~behavior_t()  = default;

        bool is_debug_enabled() const noexcept {
            return m_debug;
        }

        void enable_debug(bool debug) noexcept {
            m_debug = debug;
        }

    protected:
        bool m_debug{false};
    };
}


