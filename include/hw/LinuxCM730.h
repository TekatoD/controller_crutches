/*
 *   LinuxCM730.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _LINUX_CM730_H_
#define _LINUX_CM730_H_

#include <semaphore.h>
#include "hw/CM730.h"


namespace Robot {
    class LinuxCM730
            : public PlatformCM730 {
    public:
        LinuxCM730(const char* name);

        ~LinuxCM730();

        void SetPortName(const char* name);

        const char* GetPortName() { return (const char*) m_port_name; }

        ///////////////// Platform Porting //////////////////////
        bool OpenPort() override;

        bool SetBaud(int baud) override;

        void ClosePort() override;

        void ClearPort() override;

        int WritePort(unsigned char* packet, int numPacket) override;

        int ReadPort(unsigned char* packet, int numPacket) override;

        void LowPriorityWait() override;

        void MidPriorityWait() override;

        void HighPriorityWait() override;

        void LowPriorityRelease() override;

        void MidPriorityRelease() override;

        void HighPriorityRelease() override;

        void SetPacketTimeout(int lenPacket) override;

        bool IsPacketTimeout() override;

        double GetPacketTime() override;

        void SetUpdateTimeout(int msec) override;

        bool IsUpdateTimeout() override;

        double GetUpdateTime() override;

        void Sleep(double msec) override;

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

        sem_t m_LowSemID;
        sem_t m_MidSemID;
        sem_t m_HighSemID;
    };
}

#endif
