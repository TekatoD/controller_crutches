/**
 *  @autor tekatod
 *  @date 8/7/17
 */
#pragma once

#include <string>
#include <unordered_map>
#include <motion/JointData.h>
#include "CM730.h"

namespace Robot {

    class VrepCM730 : public CM730 {
    public:

        explicit VrepCM730(std::string device_postfix = "");

        //This is dummy for now it can do nothing
        int WriteByte(int address, int value, int* error);

        //This is dummy for now it can do nothing
        int WriteWord(int id, int address, int value, int* error);

        //This is dummy for now it can do nothing
        int ReadByte(int id, int address, int* pValue, int* error);

        //This is dummy for now it can do nothing
        bool Connect();

        //This is dummy for now it can do nothing
        bool DXLPowerOn();

        //Note: This works only with joints
        int SyncWrite(int start_addr, int each_length, int number, int *pParam);

        void DumpJoints(std::string file_name, int start_addr, int each_length, int number, int *pParam);

        int ReadWord(int id, int address, int* pValue, int* error);

        int BulkRead();

        //This is dummy for now it can do nothing
        bool ChangeBaud(int baud);

        void Disconnect();

        //This is dummy for now it can do nothing
        bool MX28InitAll();

        //This is dummy for now it can do nothing
        int WriteWord(int address, int value, int* error);

        int Ping(int id, int* error);

        //This is dummy for now it can do nothing
        int ReadTable(int id, int start_addr, int end_addr, unsigned char* table, int* error);

        //This is dummy for now it can do nothing
        int WriteByte(int id, int address, int value, int* error);

        //This is dummy for now it can do nothing
        int WriteTable(int id, int start_addr, int end_addr, unsigned char* table, int* error);

        //This is dummy for now it can do nothing
        void MakeBulkReadPacket();

        int GetClientId();

        void SetClientId(int client_id);

        ~VrepCM730();

    private:

        void init_devices();

        int connect_device(std::string device_name);

        bool m_connected;
        int m_client_id;
        std::string m_device_postfix;
        int m_joints[JointData::NUMBER_OF_JOINTS - 1];
//        std::unordered_map<int, int> m_sim_devices;
    };
}