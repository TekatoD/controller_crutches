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

#define BUFLEN 512    //Max length of buffer
#define PORT 3838    //The port on which to listen for incoming data

namespace Robot {
    class GameController {
    public:
        GameController() {
        }

        void handleOutput() {
            unsigned now = (unsigned) proxy->getTime(0);

            if (teamNumber && playerNumber &&
                *playerNumber <= gameCtrlData.playersPerTeam &&
                (gameCtrlData.teams[0].teamNumber == teamNumber ||
                 gameCtrlData.teams[1].teamNumber == teamNumber)) {
                const TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == teamNumber ? 0 : 1];
                if (gameCtrlData.state != previousState ||
                    gameCtrlData.secondaryState != previousSecondaryState ||
                    gameCtrlData.kickOffTeam != previousKickOffTeam ||
                    team.teamColour != previousTeamColour ||
                    team.players[*playerNumber - 1].penalty != previousPenalty) {
                    switch (team.teamColour) {
                        case TEAM_BLUE:
                            setLED(leftFootRed, 0.f, 0.f, 1.f);
                            break;
                        case TEAM_RED:
                            setLED(leftFootRed, 1.f, 0.f, 0.f);
                            break;
                        case TEAM_YELLOW:
                            setLED(leftFootRed, 1.f, 1.f, 0.f);
                            break;
                        default:
                            setLED(leftFootRed, 0.f, 0.f, 0.f);
                    }

                    if (gameCtrlData.state == STATE_INITIAL &&
                        gameCtrlData.secondaryState == STATE2_PENALTYSHOOT &&
                        gameCtrlData.kickOffTeam == team.teamNumber)
                        setLED(rightFootRed, 0.f, 1.f, 0.f);
                    else if (gameCtrlData.state == STATE_INITIAL &&
                             gameCtrlData.secondaryState == STATE2_PENALTYSHOOT &&
                             gameCtrlData.kickOffTeam != team.teamNumber)
                        setLED(rightFootRed, 1.f, 1.0f, 0.f);
                    else if (now - whenPacketWasReceived < GAMECONTROLLER_TIMEOUT &&
                             gameCtrlData.state <= STATE_SET &&
                             gameCtrlData.kickOffTeam == team.teamNumber)
                        setLED(rightFootRed, 1.f, 1.f, 1.f);
                    else
                        setLED(rightFootRed, 0.f, 0.f, 0.f);

                    if (team.players[*playerNumber - 1].penalty != PENALTY_NONE)
                        setLED(chestRed, 1.f, 0.f, 0.f);
                    else
                        switch (gameCtrlData.state) {
                            case STATE_READY:
                                setLED(chestRed, 0.f, 0.f, 1.f);
                                break;
                            case STATE_SET:
                                setLED(chestRed, 1.f, 1.0f, 0.f);
                                break;
                            case STATE_PLAYING:
                                setLED(chestRed, 0.f, 1.f, 0.f);
                                break;
                            default:
                                setLED(chestRed, 0.f, 0.f, 0.f);
                        }

                    ledRequest[4][0] = (int) now;
                    proxy->setAlias(ledRequest);

                    previousState = gameCtrlData.state;
                    previousSecondaryState = gameCtrlData.secondaryState;
                    previousKickOffTeam = gameCtrlData.kickOffTeam;
                    previousTeamColour = team.teamColour;
                    previousPenalty = team.players[*playerNumber - 1].penalty;
                }

                if (now - whenPacketWasReceived < GAMECONTROLLER_TIMEOUT &&
                    now - whenPacketWasSent >= ALIVE_DELAY &&
                    send(GAMECONTROLLER_RETURN_MSG_ALIVE))
                    whenPacketWasSent = now;
            }
        }

    public:
        int m_number_player;

    private:
        UdpComm m_comm;
        int playerNumber;
        int teamNumberPtr;
        int defaultTeamColour;
        in_addr gameControllerAddress;
        int teamNumber;
        RoboCupGameControlData gameCtrlData;
        
    };
}

