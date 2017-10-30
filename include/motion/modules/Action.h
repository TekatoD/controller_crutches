/*
 *   Action.h
 *
 *   Author: ROBOTIS
 *
 */

#pragma once

#include <cstdio>
#include <memory>
#include "motion/MotionModule.h"
#include "motion/JointData.h"


namespace Robot {
    class Action
            : public MotionModule {
    public:
        enum {
            MAXNUM_PAGE = 256,
            MAXNUM_STEP = 7,
            MAXNUM_NAME = 13
        };

        enum {
            SPEED_BASE_SCHEDULE = 0,
            TIME_BASE_SCHEDULE = 0x0a
        };

        enum {
            INVALID_BIT_MASK = 0x4000,
            TORQUE_OFF_BIT_MASK = 0x2000
        };

        typedef struct // Header Structure (total 64unsigned char)
        {
            unsigned char name[MAXNUM_NAME + 1]; // Name             0~13
            unsigned char reserved1;        // Reserved1        14
            unsigned char repeat;           // Repeat count     15
            unsigned char schedule;         // schedule         16
            unsigned char reserved2[3];     // reserved2        17~19
            unsigned char stepnum;          // Number of step   20
            unsigned char reserved3;        // reserved3        21
            unsigned char speed;            // Speed            22
            unsigned char reserved4;        // reserved4        23
            unsigned char accel;            // Acceleration time 24
            unsigned char next;             // Link to next     25
            unsigned char exit;             // Link to exit     26
            unsigned char reserved5[4];     // reserved5        27~30
            unsigned char checksum;         // checksum         31
            unsigned char slope[31];        // CW/CCW compliance slope  32~62, unused in latest MX-28 firmwares
            unsigned char reserved6;        // reserved6        63
        } PAGEHEADER;

        typedef struct // Step Structure (total 64unsigned char)
        {
            unsigned short position[31];    // Joint position   0~61
            unsigned char pause;            // Pause time       62
            unsigned char time;             // Time             63
        } STEP;

        typedef struct // Page Structure (total 512unsigned char)
        {
            PAGEHEADER header;          // Page header  0~64
            STEP step[MAXNUM_STEP];        // Page step    65~511
        } PAGE;

        ~Action();

        static Action* GetInstance();

        void Initialize() override;

        void Process() override;

        void LoadFile(const char* filename);

        bool CreateFile(char* filename);

        bool Start(int iPage);

        bool Start(char* namePage);

        bool Start(int index, PAGE* pPage);

        void Stop();

        void Brake();

        bool IsRunning();

        bool IsRunning(int* iPage, int* iStep);

        bool LoadPage(int index, PAGE* pPage);

        bool SavePage(int index, PAGE* pPage);

        void ResetPage(PAGE* pPage);

        bool IsDebugEnabled() const noexcept;

        void EnableDebug(bool debug) noexcept;

    private:
        Action();

        bool VerifyChecksum(PAGE* pPage);

        void SetChecksum(PAGE* pPage);

    private:
        FILE* m_action_file;
        PAGE m_play_page;
        PAGE m_next_play_page;

        int m_index_playing_page;
        bool m_first_driving_start;
        int m_page_step_count;
        bool m_playing;
        bool m_stop_playing;
        bool m_playing_finished;

        bool m_debug{false};
    };
}
