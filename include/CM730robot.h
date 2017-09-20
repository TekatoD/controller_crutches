/**
 *  @autor tekatod
 *  @date 9/20/17
 */
#pragma once

/*
 *   CM730robot.h
*   This class is the secondary controler that controls the motors MX28
 *   Author: ROBOTIS
 *
 */

#include "CM730.h"

#define MAXNUM_TXPARAM      (256)
#define MAXNUM_RXPARAM      (1024)

namespace Robot {
    
    class CM730robot : public CM730 {

    private:
        PlatformCM730* m_Platform;
        static const int RefreshTime = 6; //msec
        unsigned char m_ControlTable[MAXNUM_ADDRESS];

        unsigned char m_BulkReadTxPacket[MAXNUM_TXPARAM + 10];

        int TxRxPacket(unsigned char* txpacket, unsigned char* rxpacket, int priority);

        unsigned char CalculateChecksum(unsigned char* packet);

    public:
        bool DEBUG_PRINT;
//        BulkReadData m_BulkReadData[ID_BROADCAST];

        CM730robot(PlatformCM730* platform);

        ~CM730robot();

/*this method is to be used first to connect to the robot. Returns true when success and false when fail*/
        bool Connect();

        bool ChangeBaud(int baud);

        void Disconnect();

        bool DXLPowerOn();

        bool MX28InitAll();

        // For board
        int WriteByte(int address, int value, int* error);

        int WriteWord(int address, int value, int* error);

        // For actuators
        int Ping(int id, int* error);

        int ReadByte(int id, int address, int* pValue, int* error);

        int ReadWord(int id, int address, int* pValue, int* error);

        int ReadTable(int id, int start_addr, int end_addr, unsigned char* table, int* error);

        int WriteByte(int id, int address, int value, int* error);

        int WriteWord(int id, int address, int value, int* error);

        int WriteTable(int id, int start_addr, int end_addr, unsigned char* table, int* error);

        // For motion control
        int SyncWrite(int start_addr, int each_length, int number, int* pParam);

        void MakeBulkReadPacket();

        int BulkRead();
    };
}