/**
 *  @autor arssivka
 *  @date 4/21/17
 */

#include "gamecontroller/GameController.h"


using namespace Robot;

GameController* GameController::GetInstance() {
    static GameController gameController;
    return &gameController;
}


void GameController::Reset() {
    memset(&m_GameControllerAddress, 0, sizeof(m_GameControllerAddress));
    memset(&GameCtrlData, 0, sizeof(GameCtrlData));

    m_WhenPacketWasReceived = 0;
    m_WhenPacketWasSent = 0;
}


void GameController::Update() {
    clock_t now = clock();

    if (Receive()) {
        m_WhenPacketWasReceived = now;
    }

    if(now - m_WhenPacketWasReceived < GAMECONTROLLER_TIMEOUT &&
       now - m_WhenPacketWasSent >= ALIVE_DELAY) {
        SendAlive();
    }
}


bool GameController::GameControllerNotResponding() const {
    clock_t now = clock();
    return now - m_WhenPacketWasReceived < GAMECONTROLLER_TIMEOUT;
}


bool GameController::Connect() {
    if (m_Udp) return true;
    m_Udp = new UdpComm();
    if (m_Udp->setBlocking(false) &&
        m_Udp->setBroadcast(true) &&
        m_Udp->bind("0.0.0.0", GAMECONTROLLER_DATA_PORT) &&
        m_Udp->setLoopback(false)) {
        return true;
    }
    delete m_Udp;
    return false;
}


void GameController::Disconnect() {
    if (m_Udp) delete m_Udp;
}


bool GameController::Connected() const {
    return (bool) m_Udp;
}


void GameController::SendPenalise() {
    if (Send(GAMECONTROLLER_RETURN_MSG_MAN_PENALISE)) {
        m_WhenPacketWasSent = clock();
    }
}


void GameController::SendUnpenalise() {
    if (Send(GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE)) {
        m_WhenPacketWasSent = clock();
    }
}


void GameController::SendAlive() {
    if (Send(GAMECONTROLLER_RETURN_MSG_ALIVE)) {
        m_WhenPacketWasSent = clock();
    }
}


void GameController::LoadINISettings(minIni* ini) {
    LoadINISettings(ini, GAME_CONTROLLER_SECTION);
}


void GameController::LoadINISettings(minIni* ini, const std::string& section) {
    int value = INVALID_VALUE;
    if ((value = ini->geti(section, "team_number", INVALID_VALUE)) != INVALID_VALUE) TeamNumber = value;
    if ((value = ini->geti(section, "player_number", INVALID_VALUE)) != INVALID_VALUE) PlayerNumber = value;
}


void GameController::SaveINISettings(minIni* ini) {
    SaveINISettings(ini, GAME_CONTROLLER_SECTION);
}


void GameController::SaveINISettings(minIni* ini, const std::string& section) {
    ini->put(section, "team_number", TeamNumber);
    ini->put(section, "player_number", PlayerNumber);
}

int GameController::GetPlayerNumber() const {
    return PlayerNumber;
}

void GameController::SetPlayerNumber(int player_number) {
    PlayerNumber = player_number;
}

int GameController::GetTeamNumber() const {
    return TeamNumber;
}

void GameController::SetTeamNumber(int team_number) {
    TeamNumber = team_number;
}


GameController::~GameController() {
    Disconnect();
}


GameController::GameController(int playerNumber, int teamNumber)
        : PlayerNumber(playerNumber),
          TeamNumber(teamNumber) {
    Reset();
}


bool GameController::Send(uint8_t message) {
    RoboCupGameControlReturnData returnPacket;
    returnPacket.team = (uint8_t) TeamNumber;
    returnPacket.player = (uint8_t) PlayerNumber;
    returnPacket.message = message;
    return !m_Udp || m_Udp->write((const char*) &returnPacket, sizeof(returnPacket));
}


bool GameController::Receive() {
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
