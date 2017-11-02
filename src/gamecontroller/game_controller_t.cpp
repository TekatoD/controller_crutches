/**
 *  @autor arssivka
 *  @date 4/21/17
 */

#include <log/trivial_logger_t.h>
#include "gamecontroller/game_controller_t.h"


using namespace drwn;

game_controller_t* game_controller_t::get_instance() {
    static game_controller_t gameController;
    return &gameController;
}


void game_controller_t::reset() {
    memset(&m_game_controller_address, 0, sizeof(m_game_controller_address));
    memset(&game_ctrl_data, 0, sizeof(game_ctrl_data));

    m_when_packet_was_received = 0;
    m_when_packet_was_sent = 0;
}


void game_controller_t::update() {
    clock_t now = clock();

    if (receive()) {
        m_when_packet_was_received = now;
    }

    if(now - m_when_packet_was_received < GAMECONTROLLER_TIMEOUT &&
       now - m_when_packet_was_sent >= ALIVE_DELAY) {
        send_alive();
    }
}


bool game_controller_t::game_controller_not_responding() const {
    clock_t now = clock();
    return now - m_when_packet_was_received < GAMECONTROLLER_TIMEOUT;
}


bool game_controller_t::connect() {
    if (m_udp) return true;
    m_udp = new udp_comm_t();
    if (m_udp->set_blocking(false) &&
            m_udp->set_broadcast(true) &&
        m_udp->bind("0.0.0.0", GAMECONTROLLER_DATA_PORT) &&
            m_udp->set_loopback(false)) {
        return true;
    }
    delete m_udp;
    return false;
}


void game_controller_t::disconnect() {
    if (m_udp) delete m_udp;
}


bool game_controller_t::connected() const {
    return (bool) m_udp;
}


void game_controller_t::send_penalise() {
    if (send(GAMECONTROLLER_RETURN_MSG_MAN_PENALISE)) {
        m_when_packet_was_sent = clock();
    }
}


void game_controller_t::send_unpenalise() {
    if (send(GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE)) {
        m_when_packet_was_sent = clock();
    }
}


void game_controller_t::send_alive() {
    if (send(GAMECONTROLLER_RETURN_MSG_ALIVE)) {
        m_when_packet_was_sent = clock();
    }
}


int game_controller_t::get_player_number() const {
    return player_number;
}

void game_controller_t::set_player_number(int player_number) {
    if(m_debug) {
        LOG_DEBUG << "GAME CONTROLLER: player_number = " << player_number;
    }
    player_number = player_number;
}

int game_controller_t::get_team_number() const {
    return team_number;
}

void game_controller_t::set_team_number(int team_number) {
    if(m_debug) {
        LOG_DEBUG << "GAME CONTROLLER: team_number = " << team_number;
    }
    team_number = team_number;
}


game_controller_t::~game_controller_t() {
    disconnect();
}


game_controller_t::game_controller_t(int player_num, int team_num)
        : player_number(player_num),
          team_number(team_num) {
    reset();
}


bool game_controller_t::send(uint8_t message) {
    robo_cup_game_control_return_data_t returnPacket;
    returnPacket.team = (uint8_t) team_number;
    returnPacket.player = (uint8_t) player_number;
    returnPacket.message = message;
    return !m_udp || m_udp->write((const char*) &returnPacket, sizeof(returnPacket));
}


bool game_controller_t::receive() {
    bool received = false;
    int size;
    robo_cup_game_control_data_t buffer;
    struct sockaddr_in from;
    while (m_udp && (size = m_udp->read((char*) &buffer, sizeof(buffer), from)) > 0) {
        if (size == sizeof(buffer) &&
            !std::memcmp(&buffer, GAMECONTROLLER_STRUCT_HEADER, 4) &&
            buffer.version == GAMECONTROLLER_STRUCT_VERSION &&
            team_number &&
            (buffer.teams[0].team_number == team_number ||
             buffer.teams[1].team_number == team_number)) {
            game_ctrl_data = buffer;
            if (memcmp(&m_game_controller_address, &from.sin_addr, sizeof(in_addr))) {
                memcpy(&m_game_controller_address, &from.sin_addr, sizeof(in_addr));
                m_udp->set_target(inet_ntoa(m_game_controller_address), GAMECONTROLLER_RETURN_PORT);
            }
            received = true;
        }
    }
    return received;
}

bool game_controller_t::is_debug_enabled() const {
    return m_debug;
}

void game_controller_t::enable_debug(bool debug) {
    m_debug= debug;
}
