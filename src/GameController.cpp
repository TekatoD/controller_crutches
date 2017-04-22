/**
 *  @autor arssivka
 *  @date 4/21/17
 */

#include "GameController.h"


Robot::GameController* Robot::GameController::GetInstance() {
    static GameController gameController;
    return &gameController;
}


void Robot::GameController::Reset() {
    using namespace std::chrono;
    memset(&m_GameControllerAddress, 0, sizeof(m_GameControllerAddress));
    memset(&GameCtrlData, 0, sizeof(GameCtrlData));

    m_WhenPacketWasReceived = steady_clock::time_point();
    m_WhenPacketWasSent = steady_clock::time_point();
}


void Robot::GameController::Update() {
    using namespace std::chrono;
    TimePoint now = steady_clock::now();

    if (Receive()) {
        m_WhenPacketWasReceived = now;
    }

    if(now - m_WhenPacketWasReceived < std::chrono::milliseconds(GAMECONTROLLER_TIMEOUT) &&
       now - m_WhenPacketWasSent >= std::chrono::milliseconds(ALIVE_DELAY)) {
        SendAlive();
    }
}


bool Robot::GameController::GameControllerNotResponding() const {
    using namespace std::chrono;
    TimePoint now = steady_clock::now();
    return now - m_WhenPacketWasReceived < milliseconds(GAMECONTROLLER_TIMEOUT);
}


bool Robot::GameController::Connect() {
    if (m_Udp) return true;
    m_Udp = std::make_unique<UdpComm>();
    if (m_Udp->setBlocking(false) &&
        m_Udp->setBroadcast(true) &&
        m_Udp->bind("0.0.0.0", GAMECONTROLLER_DATA_PORT) &&
        m_Udp->setLoopback(false)) {
        return true;
    }
    m_Udp.release();
    return false;
}


void Robot::GameController::Disconnect() {
    if (m_Udp) m_Udp.release();
}


bool Robot::GameController::Connected() const {
    return (bool) m_Udp;
}


void Robot::GameController::SendPenalise() {
    using namespace std::chrono;
    if (Send(GAMECONTROLLER_RETURN_MSG_MAN_PENALISE)) {
        m_WhenPacketWasSent = steady_clock::now();
    }
}


void Robot::GameController::SendUnpenalise() {
    using namespace std::chrono;
    if (Send(GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE)) {
        m_WhenPacketWasSent = steady_clock::now();
    }
}


void Robot::GameController::SendAlive() {
    using namespace std::chrono;
    if (Send(GAMECONTROLLER_RETURN_MSG_ALIVE)) {
        m_WhenPacketWasSent = steady_clock::now();
    }
}


void Robot::GameController::LoadINISettings(minIni* ini) {
    LoadINISettings(ini, GAME_CONTROLLER_SECTION);
}


void Robot::GameController::LoadINISettings(minIni* ini, const std::string& section) {
    int value = INVALID_VALUE;
    if ((value = ini->geti(section, "team_number", INVALID_VALUE)) != INVALID_VALUE) TeamNumber = value;
    if ((value = ini->geti(section, "player_number", INVALID_VALUE)) != INVALID_VALUE) PlayerNumber = value;
}


void Robot::GameController::SaveINISettings(minIni* ini) {
    SaveINISettings(ini, GAME_CONTROLLER_SECTION);
}


void Robot::GameController::SaveINISettings(minIni* ini, const std::string& section) {
    ini->put(section, "team_number", TeamNumber);
    ini->put(section, "player_number", PlayerNumber);
}


Robot::GameController::~GameController() {
    Disconnect();
}


Robot::GameController::GameController(int playerNumber, int teamNumber)
        : PlayerNumber(playerNumber),
          TeamNumber(teamNumber) {
    Reset();
}


bool Robot::GameController::Send(uint8_t message) {
    RoboCupGameControlReturnData returnPacket;
    returnPacket.team = (uint8_t) TeamNumber;
    returnPacket.player = (uint8_t) PlayerNumber;
    returnPacket.message = message;
    return !m_Udp || m_Udp->write((const char*) &returnPacket, sizeof(returnPacket));
}


bool Robot::GameController::Receive() {
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
