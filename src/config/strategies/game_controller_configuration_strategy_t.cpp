/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#include <gamecontroller/game_controller_t.h>
#include "config/strategies/game_controller_configuration_strategy_t.h"

using namespace drwn;

game_controller_configuration_strategy_t::game_controller_configuration_strategy_t(std::string section)
        : configuration_strategy_t(std::move(section)) { }

void game_controller_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop) {
    if (prop.count(get_section()) == 0) return; // Section doesn't exist
    game_controller_t* gameController = game_controller_t::get_instance();
    auto& game_controller_section = prop.get_child(DEFAULT_SECTION);

    auto team_number = game_controller_section.get_optional<int>("team_number");
    auto player_number = game_controller_section.get_optional<int>("player_number");

    if (team_number) gameController->set_team_number(team_number.get());
    if (player_number) gameController->set_player_number(player_number.get());
}

void game_controller_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const {
    if (prop.count(get_section()) == 0) prop.add_child(get_section(), {});
    game_controller_t* gameController = game_controller_t::get_instance();
    auto& game_controller_section = prop.get_child(DEFAULT_SECTION);

    game_controller_section.put("team_number", gameController->get_team_number());
    game_controller_section.put("player_number", gameController->get_player_number());
}
