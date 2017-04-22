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
        GameController(int playerNumber = 0, int teamNumber = 0)
                : PlayerNumber(playerNumber),
                  TeamNumber(teamNumber) {
            Reset();
        }


        bool Send(uint8_t message) {
            RoboCupGameControlReturnData returnPacket;
            returnPacket.team = (uint8_t) TeamNumber;
            returnPacket.player = (uint8_t) PlayerNumber;
            returnPacket.message = message;
            return !m_Udp || m_Udp->write((const char*) &returnPacket, sizeof(returnPacket));
        }

        bool Receive() {
            bool received = false;
            int size;
            RoboCupGameControlData buffer;
            struct sockaddr_in from;
            while (m_Udp && (size = m_Udp->read((char*) &buffer, sizeof(buffer), from)) > 0) {
                if (size == sizeof(buffer) &&
                    !std::memcmp(&buffer, GAMECONTROLLER_STRUCT_HEADER, 4) &&
                    buffer.version == GAMECONTROLLER_STRUCT_VERSION &&
                    TeamNumber &&
                    (buffer.teams[0].teamNumber == TeamNumber ||
                     buffer.teams[1].teamNumber == TeamNumber)) {
                    GameCtrlData = buffer;
                    if (memcmp(&m_GameControllerAddress, &from.sin_addr, sizeof(in_addr))) {
                        memcpy(&m_GameControllerAddress, &from.sin_addr, sizeof(in_addr));
                        m_Udp->setTarget(inet_ntoa(m_GameControllerAddress), GAMECONTROLLER_RETURN_PORT);
                    }
                    received = true;
                }
            }
            return received;
        }


    private:
        std::unique_ptr<UdpComm> m_Udp;
        in_addr m_GameControllerAddress;
        TimePoint m_WhenPacketWasReceived;
        TimePoint m_WhenPacketWasSent;

    };
}

