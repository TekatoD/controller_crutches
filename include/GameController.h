/**
 *  @autor arssivka
 *  @date 4/21/17
 */

#pragma once


#include "gamecontroller/RoboCupGameControlData.h"
#include "gamecontroller/SPLStandardMessage.h"
#include "gamecontroller/SPLCoachMessage.h"
#include "UdpComm.h"
#include "minIni.h"
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstring>
#include <stdexcept>
#include <chrono>

#define GAME_CONTROLLER_SECTION   "Game Controller"
#define INVALID_VALUE   -1024.0

static const int GAMECONTROLLER_TIMEOUT = 2000;
static const int ALIVE_DELAY = 1000;

namespace Robot {
    class GameController {
    public:
        typedef std::chrono::steady_clock::time_point TimePoint;

        static GameController* GetInstance();

        void Reset();

        void Update();

        bool GameControllerNotResponding() const noexcept;

        bool Connect();

        void Disconnect();

        bool Connected() const noexcept;

        void SendPenalise();

        void SendUnpenalise();

        void SendAlive();

        void LoadINISettings(minIni* ini);

        void LoadINISettings(minIni* ini, const std::string& section);

        void SaveINISettings(minIni* ini);

        void SaveINISettings(minIni* ini, const std::string& section);

        ~GameController();


    public:
        int PlayerNumber;
        int TeamNumber;
        RoboCupGameControlData GameCtrlData;

    private:
        GameController(int playerNumber = 0, int teamNumber = 0);

        bool Send(uint8_t message);

        bool Receive();


    private:
        std::unique_ptr<UdpComm> m_Udp;
        in_addr m_GameControllerAddress;
        TimePoint m_WhenPacketWasReceived;
        TimePoint m_WhenPacketWasSent;

    };
}

