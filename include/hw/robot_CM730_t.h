/**
 *  @autor tekatod
 *  @date 9/20/17
 */
#pragma once

/*
 *   robot_CM730_t.h
*   This class is the secondary controler that controls the motors MX28
 *   Author: ROBOTIS
 *
 */

#include "CM730_t.h"

#define MAXNUM_TXPARAM      (256)
#define MAXNUM_RXPARAM      (1024)

namespace drwn {
    
    class robot_CM730_t : public CM730_t {

    private:
        platform_CM730_t* m_platform;
        static const int refresh_time = 6; //msec
        unsigned char m_control_table[MAXNUM_ADDRESS];

        unsigned char m_bulk_read_tx_packet[MAXNUM_TXPARAM + 10];

        int TxRx_packet(unsigned char* txpacket, unsigned char* rxpacket, int priority);

        unsigned char calculate_checksum(unsigned char* packet);

    public:
        bool DEBUG_PRINT;
//        bulk_read_data_t m_bulk_read_data[ID_BROADCAST];

        robot_CM730_t(platform_CM730_t* platform);

        ~robot_CM730_t();

/*this method is to be used first to connect to the robot. Returns true when success and false when fail*/
        bool connect();

        bool change_baud(int baud);

        void disconnect();

        bool DXL_power_on();

        bool MX28_init_all();

        // For board
        int write_byte(int address, int value, int* error);

        int write_word(int address, int value, int* error);

        // For actuators
        int ping(int id, int* error);

        int read_byte(int id, int address, int* pValue, int* error);

        int read_word(int id, int address, int* pValue, int* error);

        int read_table(int id, int start_addr, int end_addr, unsigned char* table, int* error);

        int write_byte(int id, int address, int value, int* error);

        int write_word(int id, int address, int value, int* error);

        int write_table(int id, int start_addr, int end_addr, unsigned char* table, int* error);

        // For motion control
        int sync_write(int start_addr, int each_length, int number, int* pParam);

        void make_bulk_read_packet();

        int bulk_read();
    };
}