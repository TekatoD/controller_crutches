/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#include <motion/modules/Walking.h>
#include <motion/MotionManager.h>
#include "config/BallSearcherConfigurationStrategy.h"

using namespace Robot;

BallSearcherConfigurationStrategy::BallSearcherConfigurationStrategy(BallSearcher* ballSearcher, std::string section)
        : ConfigurationStrategy(std::move(section)), m_ball_searcher(ballSearcher) { }

void BallSearcherConfigurationStrategy::ReadConfig(const boost::property_tree::ptree& prop) {
    if (m_ball_searcher != nullptr) {
        if (prop.count(GetSection()) == 0) return; // Section doesn't exist

        auto& searcher_section = prop.get_child(this->GetSection());
        auto tilt_phase_step = searcher_section.get_optional<float>("tilt_phase_step");
        auto pan_phase_step = searcher_section.get_optional<float>("pan_phase_step");
        auto phase_size = searcher_section.get_optional<float>("phase_size");
        auto turn_step = searcher_section.get_optional<float>("turn_step");
        auto max_turn = searcher_section.get_optional<float>("max_turn");

        if (tilt_phase_step) m_ball_searcher->SetTiltPhaseStep(tilt_phase_step.get());
        if (pan_phase_step) m_ball_searcher->SetPanPhaseStep(pan_phase_step.get());
        if (phase_size ) m_ball_searcher->SetPhaseSize(phase_size .get());
        if (turn_step) m_ball_searcher->SetTurnStep(turn_step.get());
        if (max_turn) m_ball_searcher->SetMaxTurn(max_turn.get());
    }
    else {
        throw std::runtime_error("Ball Searcher configuration load fail: BallSearcher nullptr");
    }
}

void BallSearcherConfigurationStrategy::WriteConfig(boost::property_tree::ptree& prop) const {
    if (m_ball_searcher != nullptr) {
        if (prop.count(GetSection()) == 0) prop.add_child(GetSection(), {});

        auto& searcher_section = prop.get_child(this->GetSection());
        searcher_section.put("tilt_phase_step", m_ball_searcher->GetTiltPhaseStep());
        searcher_section.put("pan_phase_step", m_ball_searcher->GetPanPhaseStep());
        searcher_section.put("phase_size", m_ball_searcher->GetPhaseSize());
        searcher_section.put("turn_step", m_ball_searcher->GetTurnStep());
        searcher_section.put("max_turn", m_ball_searcher->GetMaxTurn());
    }
    else {
        throw std::runtime_error("Ball Searcher configuration write fail: BallSearcher nullptr");
    }
}