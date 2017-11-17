/**
 *  @autor tekatod
 *  @date 8/7/17
 */
#pragma once

#include <string>
#include <unordered_map>
#include <motion/joint_data_t.h>
#include "CM730_t.h"

namespace drwn {

    class vrep_CM730_t : public CM730_t {
    public:

        explicit vrep_CM730_t(std::string device_postfix = "");

        //This is dummy for now it can do nothing
        int write_byte(int address, int value, int* error);

        //This is dummy for now it can do nothing
        int write_word(int id, int address, int value, int* error);

        //This is dummy for now it can do nothing
        int read_byte(int id, int address, int* pValue, int* error);

        //This is dummy for now it can do nothing
        bool connect();

        //This is dummy for now it can do nothing
        bool DXL_power_on();

        //Note: This works only with joints
        int sync_write(int start_addr, int each_length, int number, int* pParam);

        int read_word(int id, int address, int* pValue, int* error);

        int bulk_read();

        //This is dummy for now it can do nothing
        bool change_baud(int baud);

        void disconnect();

        //This is dummy for now it can do nothing
        bool MX28_init_all();

        //This is dummy for now it can do nothing
        int write_word(int address, int value, int* error);

        int ping(int id, int* error);

        //This is dummy for now it can do nothing
        int read_table(int id, int start_addr, int end_addr, unsigned char* table, int* error);

        //This is dummy for now it can do nothing
        int write_byte(int id, int address, int value, int* error);

        //This is dummy for now it can do nothing
        int write_table(int id, int start_addr, int end_addr, unsigned char* table, int* error);

        //This is dummy for now it can do nothing
        void make_bulk_read_packet();

        int get_client_id();

        void set_client_id(int client_id);

        ~vrep_CM730_t();

    private:

        void init_devices();

        int connect_device(std::string device_name);

        bool m_connected;
        int m_client_id;
        std::string m_device_postfix;
        int m_joints[joint_data_t::NUMBER_OF_JOINTS - 1];
//        std::unordered_map<int, int> m_sim_devices;
    };
}