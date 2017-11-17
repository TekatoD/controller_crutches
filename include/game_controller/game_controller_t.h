/**
 *  @autor arssivka
 *  @date 4/21/17
 */

#pragma once


#include "game_controller/robo_cup_game_control_data_t.h"
#include "game_controller/spl_standard_message.h"
#include "game_controller/spl_coach_message.h"
#include "udp_comm_t.h"
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cstring>
#include <stdexcept>

namespace drwn {
    class game_controller_t {
    public:
        static constexpr int GAMECONTROLLER_TIMEOUT = 2000000;
        static constexpr int ALIVE_DELAY = 1000000;

        static game_controller_t* get_instance();

        void reset();

        void update();

        bool not_responding() const;

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

        const robo_cup_game_control_data_t& get_game_ctrl_data() const noexcept;

    private:
        explicit game_controller_t(int player_num = 0, int team_num = 0);

        bool send(uint8_t message);

        bool receive();


    private:
        int m_player_number;
        int m_team_number;
        robo_cup_game_control_data_t m_game_ctrl_data;

        udp_comm_t* m_udp;
        in_addr m_game_controller_address;
        clock_t m_when_packet_was_received;
        clock_t m_when_packet_was_sent;
        bool m_debug{false};

    };
}

