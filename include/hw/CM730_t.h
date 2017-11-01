/**
 *  @autor tekatod
 *  @date 9/20/17
 */
#pragma once

#include "MX28_t.h"

#define MAXNUM_TXPARAM      (256)
#define MAXNUM_RXPARAM      (1024)

namespace drwn {
    class bulk_read_data_t {
    public:
        int start_address;
        int length;
        int error;
        unsigned char table[MX28_t::MAXNUM_ADDRESS];

        bulk_read_data_t();

        int read_byte(int address);

        int read_word(int address);
    };

/*
platform_CM730_t

This abstract class corresponds to a platform CM730_t that is the secondary controler that controls the motors MX28
For instance, linux_CM730_t is a concretisation of this class.
*/
    class platform_CM730_t {
    public:
        /////////// Need to implement below methods (Platform porting) //////////////
        // Port control
        virtual bool open_port() = 0;

        virtual bool set_baud(int baud) = 0;

        virtual void close_port() = 0;

        virtual void clear_port() = 0;

        virtual int write_port(unsigned char* packet, int num_packet) = 0;

        virtual int read_port(unsigned char* packet, int num_packet) = 0;

        // Using semaphore
        virtual void low_priority_wait() = 0;

        virtual void mid_priority_wait() = 0;

        virtual void high_priority_wait() = 0;

        virtual void low_priority_release() = 0;

        virtual void mid_priority_release() = 0;

        virtual void high_priority_release() = 0;

        // Using timeout
        virtual void set_packet_timeout(int len_packet) = 0;

        virtual bool is_packet_timeout() = 0;

        virtual double get_packet_time() = 0;

        virtual void set_update_timeout(int msec) = 0;

        virtual bool is_update_timeout() = 0;

        virtual double get_update_time() = 0;

        virtual void sleep(double msec) = 0;
        //////////////////////////////////////////////////////////////////////////////
    };

    class CM730_t {
    public:
        enum {
            SUCCESS,
            TX_CORRUPT,
            TX_FAIL,
            RX_FAIL,
            RX_TIMEOUT,
            RX_CORRUPT
        };

        enum {
            INPUT_VOLTAGE = 1,
            ANGLE_LIMIT = 2,
            OVERHEATING = 4,
            RANGE = 8,
            CHECKSUM = 16,
            OVERLOAD = 32,
            INSTRUCTION = 64
        };

        /*EEPROM and RAM p. 4 in MX28 Technical Specifications.pdf ????*/
        enum {
            P_MODEL_NUMBER_L = 0,
            P_MODEL_NUMBER_H = 1,
            P_VERSION = 2,
            P_ID = 3,
            P_BAUD_RATE = 4,
            P_RETURN_DELAY_TIME = 5,
            P_RETURN_LEVEL = 16,
            P_DXL_POWER = 24,
            P_LED_PANNEL = 25,
            P_LED_HEAD_L = 26,
            P_LED_HEAD_H = 27,
            P_LED_EYE_L = 28,
            P_LED_EYE_H = 29,
            P_BUTTON = 30,
            P_GYRO_Z_L = 38,
            P_GYRO_Z_H = 39,
            P_GYRO_Y_L = 40,
            P_GYRO_Y_H = 41,
            P_GYRO_X_L = 42,
            P_GYRO_X_H = 43,
            P_ACCEL_X_L = 44,
            P_ACCEL_X_H = 45,
            P_ACCEL_Y_L = 46,
            P_ACCEL_Y_H = 47,
            P_ACCEL_Z_L = 48,
            P_ACCEL_Z_H = 49,
            P_VOLTAGE = 50,
            P_LEFT_MIC_L = 51,
            P_LEFT_MIC_H = 52,
            P_ADC2_L = 53,
            P_ADC2_H = 54,
            P_ADC3_L = 55,
            P_ADC3_H = 56,
            P_ADC4_L = 57,
            P_ADC4_H = 58,
            P_ADC5_L = 59,
            P_ADC5_H = 60,
            P_ADC6_L = 61,
            P_ADC6_H = 62,
            P_ADC7_L = 63,
            P_ADC7_H = 64,
            P_ADC8_L = 65,
            P_ADC8_H = 66,
            P_RIGHT_MIC_L = 67,
            P_RIGHT_MIC_H = 68,
            P_ADC10_L = 69,
            P_ADC10_H = 70,
            P_ADC11_L = 71,
            P_ADC11_H = 72,
            P_ADC12_L = 73,
            P_ADC12_H = 74,
            P_ADC13_L = 75,
            P_ADC13_H = 76,
            P_ADC14_L = 77,
            P_ADC14_H = 78,
            P_ADC15_L = 79,
            P_ADC15_H = 80,
            MAXNUM_ADDRESS
        };

        enum {
            ID_CM = 200,
            ID_BROADCAST = 254
        };

    public:

        bulk_read_data_t m_bulk_read_data[ID_BROADCAST];

//        virtual ~CM730_t() = 0;

/*this method is to be used first to connect to the robot. Returns true when success and false when fail*/
        virtual bool connect() = 0;

        virtual bool change_baud(int baud) = 0;

        virtual void disconnect() = 0;

        virtual bool DXL_power_on() = 0;

        virtual bool MX28_init_all() = 0;

        // For board
        virtual int write_byte(int address, int value, int* error) = 0;

        virtual int write_word(int address, int value, int* error) = 0;

        // For actuators
        virtual int ping(int id, int* error) = 0;

        virtual int read_byte(int id, int address, int* pValue, int* error) = 0;

        virtual int read_word(int id, int address, int* pValue, int* error) = 0;

        virtual int read_table(int id, int start_addr, int end_addr, unsigned char* table, int* error) = 0;

        virtual int write_byte(int id, int address, int value, int* error) = 0;

        virtual int write_word(int id, int address, int value, int* error) = 0;

        virtual int write_table(int id, int start_addr, int end_addr, unsigned char* table, int* error) = 0;

        // For motion control
        virtual int sync_write(int start_addr, int each_length, int number, int* pParam) = 0;

        virtual void make_bulk_read_packet() = 0;

        virtual int bulk_read() = 0;

        // Utility
        static int make_word(int lowbyte, int highbyte);

        static int get_low_byte(int word);

        static int get_high_byte(int word);

        static int make_color(int red, int green, int blue);
    };
}
