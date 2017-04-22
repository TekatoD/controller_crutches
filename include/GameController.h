/**
 *  @autor arssivka
 *  @date 4/21/17
 */

#pragma once


#include "gamecontroller/RoboCupGameControlData.h"
#include "gamecontroller/SPLStandardMessage.h"
#include "gamecontroller/SPLCoachMessage.h"
#include "UdpComm.h"
#include <stdio.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <chrono>

static const int GAMECONTROLLER_TIMEOUT = 2000;
static const int ALIVE_DELAY = 1000;

namespace Robot {
    class GameController {
    public:
        typedef std::chrono::steady_clock::time_point TimePoint;

        GameController(int playerNumber = 0, int teamNumber = 0)
                : PlayerNumber(playerNumber),
                  TeamNumber(teamNumber) {
            Reset();
        }

        void Reset() {
            using namespace std::chrono;
            memset(&m_GameControllerAddress, 0, sizeof(m_GameControllerAddress));
            memset(&GameCtrlData, 0, sizeof(GameCtrlData));

            m_WhenPacketWasReceived = steady_clock::time_point();
            m_WhenPacketWasSent = steady_clock::time_point();
        }

        void Update() {
            using namespace std::chrono;
            TimePoint now = steady_clock::now();

            if (Receive()) {
                m_WhenPacketWasReceived = now;
            }

            if (now - m_WhenPacketWasReceived < milliseconds(GAMECONTROLLER_TIMEOUT) &&
                    now - m_WhenPacketWasSent >= milliseconds(ALIVE_DELAY) &&
                    Send(GAMECONTROLLER_RETURN_MSG_ALIVE)) {
                m_WhenPacketWasSent = now;
            }
        }

        bool GameControllerNotResponding() const noexcept {
            using namespace std::chrono;
            TimePoint now = steady_clock::now();
            return now - m_WhenPacketWasReceived < milliseconds(GAMECONTROLLER_TIMEOUT);
        }


        void Connect() {
            if (m_Udp) return;
            m_Udp = std::make_unique<UdpComm>();
            if (!m_Udp->setBlocking(false) ||
                    !m_Udp->setBroadcast(true) ||
                    !m_Udp->bind("0.0.0.0", GAMECONTROLLER_DATA_PORT) ||
                    !m_Udp->setLoopback(false)) {
                m_Udp.release();
                std::cerr << "ERROR: Can't open UPD port!" << std::endl;
            }
        }

        void Disconnect() {
            if (m_Udp) m_Udp.release();
        }

        bool Connected() const noexcept {
            return (bool) m_Udp;
        }

        void SendPenalise() {
            using namespace std::chrono;
            if (Send(GAMECONTROLLER_RETURN_MSG_MAN_PENALISE)) {
                m_WhenPacketWasSent = steady_clock::now();
            }
        }

        void SendUnpenalise() {
            using namespace std::chrono;
            if (Send(GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE)) {
                m_WhenPacketWasSent = steady_clock::now();
            }
        }


    public:
        int PlayerNumber;
        int TeamNumber;
        RoboCupGameControlData GameCtrlData;


    private:
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

