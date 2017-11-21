/**
 *  @autor arssivka
 *  @date 4/21/17
 */

#include <log/trivial_logger_t.h>
#include "game_controller/game_controller_t.h"


using namespace drwn;

game_controller_t* game_controller_t::get_instance() {
    static game_controller_t instance;
    return &instance;
}


void game_controller_t::reset() {
    memset(&m_game_controller_address, 0, sizeof(m_game_controller_address));
    memset(&m_game_ctrl_data, 0, sizeof(m_game_ctrl_data));

    m_when_packet_was_received = 0;
    m_when_packet_was_sent = 0;
}


void game_controller_t::update() {
    clock_t now = clock();

    if (this->receive()) {
        m_when_packet_was_received = now;
    }

    if (now - m_when_packet_was_received < GAMECONTROLLER_TIMEOUT &&
        now - m_when_packet_was_sent >= ALIVE_DELAY) {
        this->send_alive();
    }
}


bool game_controller_t::not_responding() const {
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
        if (m_debug) LOG_DEBUG << "GAME CONTROLLER: Server socket has been bound";
        return true;
    } else {
        if (m_debug) LOG_DEBUG << "GAME CONTROLLER: Can't bind server socket";
        delete m_udp;
        return false;
    }
}


void game_controller_t::disconnect() {
    if (m_udp != nullptr) {
        if (m_debug) LOG_DEBUG << "GAME CONTROLLER: Game controller disconnected";
        delete m_udp;
    }
}


bool game_controller_t::connected() const {
    return (bool) m_udp;
}


void game_controller_t::send_penalise() {
    if (this->send(GAMECONTROLLER_RETURN_MSG_MAN_PENALISE)) {
        if (m_debug) LOG_DEBUG << "GAME CONTROLLER: Sending manual penalise message";
        m_when_packet_was_sent = clock();
    }
}


void game_controller_t::send_unpenalise() {
    if (this->send(GAMECONTROLLER_RETURN_MSG_MAN_UNPENALISE)) {
        if (m_debug) LOG_DEBUG << "GAME CONTROLLER: Sending manual unpenalise message";
        m_when_packet_was_sent = clock();
    }
}


void game_controller_t::send_alive() {
    if (this->send(GAMECONTROLLER_RETURN_MSG_ALIVE)) {
        if (m_debug) LOG_DEBUG << "GAME CONTROLLER: Sending alive message";
        m_when_packet_was_sent = clock();
    }
}


int game_controller_t::get_player_number() const {
    return m_player_number;
}

void game_controller_t::set_player_number(int player_number) {
    if (m_debug) {
        LOG_DEBUG << "GAME CONTROLLER: player_number = " << player_number;
    }
    m_player_number = player_number;
}

int game_controller_t::get_team_number() const {
    return m_team_number;
}

void game_controller_t::set_team_number(int team_number) {
    if (m_debug) {
        LOG_DEBUG << "GAME CONTROLLER: team_number = " << team_number;
    }
    m_team_number = team_number;
}


game_controller_t::~game_controller_t() {
    this->disconnect();
}


game_controller_t::game_controller_t(int player_num, int team_num)
        : m_player_number(player_num),
          m_team_number(team_num) {
    this->reset();
}


bool game_controller_t::send(uint8_t message) {
    robo_cup_game_control_return_data_t return_packet;
    return_packet.team = (uint8_t) m_team_number;
    return_packet.player = (uint8_t) m_player_number;
    return_packet.message = message;
    auto result = !m_udp || m_udp->write((const char*) &return_packet, sizeof(return_packet));
    if (m_debug && !result) {
        LOG_DEBUG << "GAME CONTROLLER: Can't send message";
    }
    return result;
}


bool game_controller_t::receive() {
    bool received = false;
    int size;
    robo_cup_game_control_data_t buffer;
    sockaddr_in from;
    while (m_udp && (size = m_udp->read((char*) &buffer, sizeof(buffer), from)) > 0) {
        if (size == sizeof(buffer) &&
            !std::memcmp(&buffer, GAMECONTROLLER_STRUCT_HEADER, 4) &&
            buffer.version == GAMECONTROLLER_STRUCT_VERSION &&
            m_team_number &&
            (buffer.teams[0].team_number == m_team_number ||
             buffer.teams[1].team_number == m_team_number)) {
            if (m_debug) LOG_DEBUG << "GAME CONTROLLER: Message received";
            m_game_ctrl_data = buffer;
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
    m_debug = debug;
}

const robo_cup_game_control_data_t& game_controller_t::get_game_ctrl_data() const noexcept {
    return m_game_ctrl_data;
}
