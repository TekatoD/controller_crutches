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

#define GAME_CONTROLLER_SECTION   "Game Controller"
#define INVALID_VALUE   -1024.0

static const int GAMECONTROLLER_TIMEOUT = 2000000;
static const int ALIVE_DELAY = 1000000;

namespace Robot {
    class GameController {
    public:
        static GameController* GetInstance();

        void Reset();

        void Update();

        bool GameControllerNotResponding() const;

        bool Connect();

        void Disconnect();

        bool Connected() const;

        void SendPenalise();

        void SendUnpenalise();

        void SendAlive();

        void LoadINISettings(minIni* ini);

        void LoadINISettings(minIni* ini, const std::string& section);

        void SaveINISettings(minIni* ini);

        void SaveINISettings(minIni* ini, const std::string& section);

        ~GameController();

        int GetPlayerNumber() const;

        void SetPlayerNumber(int player_number);

        int GetTeamNumber() const;

        void SetTeamNumber(int team_number);


    public:
        int PlayerNumber;
        int TeamNumber;
        RoboCupGameControlData GameCtrlData;

    private:
        GameController(int playerNumber = 0, int teamNumber = 0);

        bool Send(uint8_t message);

        bool Receive();


    private:
        UdpComm* m_Udp;
        in_addr m_GameControllerAddress;
        clock_t m_WhenPacketWasReceived;
        clock_t m_WhenPacketWasSent;

    };
}

