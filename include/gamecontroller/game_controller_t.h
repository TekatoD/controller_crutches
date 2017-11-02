/**
 *  @autor arssivka
 *  @date 4/21/17
 */

#pragma once


#include "gamecontroller/robo_cup_game_control_data_t.h"
#include "gamecontroller/spl_standard_message.h"
#include "gamecontroller/spl_coach_message.h"
#include "udp_comm_t.h"
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstring>
#include <stdexcept>

#define GAME_CONTROLLER_SECTION   "Game Controller"
#define INVALID_VALUE   -1024.0

static const int GAMECONTROLLER_TIMEOUT = 2000000;
static const int ALIVE_DELAY = 1000000;

namespace drwn {
    class game_controller_t {
    public:
        static game_controller_t* get_instance();

        void reset();

        void update();

        bool game_controller_not_responding() const;

        bool connect();

        void disconnect();

        bool connected() const;

        void send_penalise();

        void send_unpenalise();

        void send_alive();

        ~game_controller_t();

        int get_player_number() const;

        void set_player_number(int player_number);

        int get_team_number() const;

        void set_team_number(int team_number);

        bool is_debug_enabled() const;

        void enable_debug(bool debug);


    public:
        int player_number;
        int team_number;
        robo_cup_game_control_data_t game_ctrl_data;

    private:
        explicit game_controller_t(int player_num = 0, int team_num = 0);

        bool send(uint8_t message);

        bool receive();


    private:
        udp_comm_t* m_udp;
        in_addr m_game_controller_address;
        clock_t m_when_packet_was_received;
        clock_t m_when_packet_was_sent;
        bool m_debug{false};

    };
}

