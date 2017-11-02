/**
 *  @autor tekatod
 *  @date 10/25/17
 */
#include <motion/modules/walking_t.h>
#include <motion/motion_manager_t.h>
#include "config/strategies/ball_searcher_configuration_strategy_t.h"

using namespace drwn;

ball_searcher_configuration_strategy_t::ball_searcher_configuration_strategy_t(ball_searcher_t* ballSearcher, std::string section)
        : configuration_strategy_t(std::move(section)), m_ball_searcher(ballSearcher) { }

void ball_searcher_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop) {
    if (m_ball_searcher != nullptr) {
        if (prop.count(get_section()) == 0) return; // Section doesn't exist

        auto& searcher_section = prop.get_child(this->get_section());
        auto tilt_phase_step = searcher_section.get_optional<float>("tilt_phase_step");
        auto pan_phase_step = searcher_section.get_optional<float>("pan_phase_step");
        auto phase_size = searcher_section.get_optional<float>("phase_size");
        auto turn_step = searcher_section.get_optional<float>("turn_step");
        auto max_turn = searcher_section.get_optional<float>("max_turn");

        if (tilt_phase_step) m_ball_searcher->set_tilt_phase_step(tilt_phase_step.get());
        if (pan_phase_step) m_ball_searcher->set_pan_phase_step(pan_phase_step.get());
        if (phase_size ) m_ball_searcher->set_phase_size(phase_size.get());
        if (turn_step) m_ball_searcher->set_turn_step(turn_step.get());
        if (max_turn) m_ball_searcher->set_max_turn(max_turn.get());
    }
    else {
        throw std::runtime_error("Ball Searcher configuration load fail: BallSearcher nullptr");
    }
}

void ball_searcher_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const {
    if (m_ball_searcher != nullptr) {
        if (prop.count(get_section()) == 0) prop.add_child(get_section(), {});

        auto& searcher_section = prop.get_child(this->get_section());
        searcher_section.put("tilt_phase_step", m_ball_searcher->get_tilt_phase_step());
        searcher_section.put("pan_phase_step", m_ball_searcher->get_pan_phase_step());
        searcher_section.put("phase_size", m_ball_searcher->get_phase_size());
        searcher_section.put("turn_step", m_ball_searcher->get_turn_step());
        searcher_section.put("max_turn", m_ball_searcher->get_max_turn());
    }
    else {
        throw std::runtime_error("Ball Searcher configuration write fail: BallSearcher nullptr");
    }
}