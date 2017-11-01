/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#include <gamecontroller/GameController.h>
#include "config/strategies/GameControllerConfigurationStrategy.h"

using namespace drwn;

GameControllerConfigurationStrategy::GameControllerConfigurationStrategy(std::string section)
        : ConfigurationStrategy(std::move(section)) { }

void GameControllerConfigurationStrategy::ReadConfig(const boost::property_tree::ptree& prop) {
    if (prop.count(GetSection()) == 0) return; // Section doesn't exist
    GameController* gameController = GameController::GetInstance();
    auto& game_controller_section = prop.get_child(DEFAULT_SECTION);

    auto team_number = game_controller_section.get_optional<int>("team_number");
    auto player_number = game_controller_section.get_optional<int>("player_number");

    if (team_number) gameController->SetTeamNumber(team_number.get());
    if (player_number) gameController->SetPlayerNumber(player_number.get());
}

void GameControllerConfigurationStrategy::WriteConfig(boost::property_tree::ptree& prop) const {
    if (prop.count(GetSection()) == 0) prop.add_child(GetSection(), {});
    GameController* gameController = GameController::GetInstance();
    auto& game_controller_section = prop.get_child(DEFAULT_SECTION);

    game_controller_section.put("team_number",gameController->GetTeamNumber());
    game_controller_section.put("player_number",gameController->GetPlayerNumber());
}
