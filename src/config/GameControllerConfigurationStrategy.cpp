/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#include <gamecontroller/GameController.h>
#include "config/GameControllerConfigurationStrategy.h"

using namespace Robot;

void GameControllerConfigurationStrategy::ReadConfig(const boost::property_tree::ptree& prop) {
    GameController* gameController = GameController::GetInstance();

    auto team_number = prop.get_optional<int>("team_number");
    auto player_number = prop.get_optional<int>("player_number");

    if (team_number) gameController->SetTeamNumber(team_number.get());
    if (player_number) gameController->SetPlayerNumber(player_number.get());
}

void GameControllerConfigurationStrategy::WriteConfig(boost::property_tree::ptree& prop) const {
    GameController* gameController = GameController::GetInstance();

    prop.put("team_number",gameController->GetTeamNumber());
    prop.put("player_number",gameController->GetPlayerNumber());
}