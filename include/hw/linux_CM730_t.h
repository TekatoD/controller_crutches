/*
 *   LinuxCM730.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once

#include <semaphore.h>
#include "hw/CM730_t.h"


namespace drwn {
    class linux_CM730_t
            : public platform_CM730_t {
    public:
        linux_CM730_t(const char* name);

        ~linux_CM730_t();

        void set_port_name(const char* name);

        const char* get_port_name() { return (const char*) m_port_name; }

        ///////////////// Platform Porting //////////////////////
        bool open_port() override;

        bool set_baud(int baud) override;

        void close_port() override;

        void clear_port() override;

        int write_port(unsigned char* packet, int numPacket) override;

        int read_port(unsigned char* packet, int numPacket) override;

        void low_priority_wait() override;

        void mid_priority_wait() override;

        void high_priority_wait() override;

        void low_priority_release() override;

        void mid_priority_release() override;

        void high_priority_release() override;

        void set_packet_timeout(int lenPacket) override;

        bool is_packet_timeout() override;

        double get_packet_time() override;

        void set_update_timeout(int msec) override;

        bool is_update_timeout() override;

        double get_update_time() override;

        void sleep(double msec) override;

        bool IsDebugEnabled() const;

        void EnableDebug(bool debug);

    private:
        double GetCurrentTime();

    private:
        bool m_debug{false};

        int m_socket_fd{-1};
        double m_packet_start_time{0};
        double m_packet_wait_time{0};
        double m_update_start_time{0};
        double m_update_wait_time{0};
        double m_byte_transfer_time{0};
        char m_port_name[20];

        sem_t m_low_sem_ID;
        sem_t m_mid_sem_ID;
        sem_t m_high_sem_ID;
    };
}
