//
// Created by akovalev on 16.11.17.
//

#include <config/strategies/particle_filter_configuration_strategy_t.h>
#include <math/angle_tools.h>

using namespace drwn;

particle_filter_configuration_strategy_t::particle_filter_configuration_strategy_t(particle_filter_t* pf_ptr, std::string section)
: configuration_strategy_t(std::move(section)), m_particle_filter(pf_ptr)
{

}

void particle_filter_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop)
{
    if (m_particle_filter == nullptr) {
        throw std::runtime_error{"Localization configuration load failure: m_particle_filter nullptr"};
    }
    if (prop.count(this->get_section()) == 0) { return; }

    auto& section = prop.get_child(this->get_section());

    auto num_particles = section.get_optional<int>("particle_number");
    auto init_x = section.get_optional<float>("init_x");
    auto init_y = section.get_optional<float>("init_y");
    auto init_theta = section.get_optional<float>("init_theta");
    auto random_particles = section.get_optional<int>("random_particles");
    auto min_x = section.get_optional<float>("min_x");
    auto min_y = section.get_optional<float>("min_y");
    auto min_theta = section.get_optional<float>("min_theta");
    auto max_x = section.get_optional<float>("max_x");
    auto max_y = section.get_optional<float>("max_y");
    auto max_theta = section.get_optional<float>("max_theta");
    auto odo_noise_rot1 = section.get_optional<float>("odo_noise_rot1");
    auto odo_noise_trans = section.get_optional<float>("odo_noise_trans");
    auto odo_noise_rot2 = section.get_optional<float>("odo_noise_rot2");
    auto meas_noise_range = section.get_optional<float>("meas_noise_range");
    auto meas_noise_bearing = section.get_optional<float>("meas_noise_bearing");
    auto loc_threshold_x = section.get_optional<float>("loc_threshold_x");
    auto loc_threshold_y = section.get_optional<float>("loc_threshold_y");
    auto loc_threshold_theta = section.get_optional<float>("loc_threshold_theta");

    if (num_particles) { m_particle_filter->set_particle_number(num_particles.get()); }
    if (init_x) { m_particle_filter->set_init_x(init_x.get()); }
    if (init_y) { m_particle_filter->set_init_y(init_y.get()); }
    if (init_theta) { m_particle_filter->set_init_theta(radians(init_theta.get())); }
    if (random_particles) { m_particle_filter->set_random_particles(random_particles.get()); }
    if (min_x) { m_particle_filter->set_min_x(min_x.get()); }
    if (min_y) { m_particle_filter->set_min_y(min_y.get()); }
    if (min_theta) { m_particle_filter->set_min_theta(radians(min_theta.get())); }
    if (max_x) { m_particle_filter->set_max_x(max_x.get()); }
    if (max_y) { m_particle_filter->set_max_y(max_y.get()); }
    if (max_theta) { m_particle_filter->set_max_theta(radians(max_theta.get())); }
    if (odo_noise_rot1) { m_particle_filter->set_odo_noise_rot1(radians(odo_noise_rot1.get())); }
    if (odo_noise_trans) { m_particle_filter->set_odo_noise_trans(odo_noise_trans.get()); }
    if (odo_noise_rot2) { m_particle_filter->set_odo_noise_rot2(radians(odo_noise_rot2.get())); }
    if (meas_noise_range) { m_particle_filter->set_meas_noise_range(radians(meas_noise_range.get())); }
    if (meas_noise_bearing) { m_particle_filter->set_meas_noise_bearing(meas_noise_bearing.get()); }
    if (loc_threshold_x) { m_particle_filter->set_loc_threshold_x(loc_threshold_x.get()); }
    if (loc_threshold_y) { m_particle_filter->set_loc_threshold_y(loc_threshold_y.get()); }
    if (loc_threshold_theta) { m_particle_filter->set_loc_threshold_theta(radians(loc_threshold_theta.get())); }


    m_particle_filter->initialize();
}

void particle_filter_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const
{
    //TODO:
}

void particle_filter_configuration_strategy_t::set_particle_filter(particle_filter_t* pf_ptr)
{
    assert(pf_ptr != nullptr);
    m_particle_filter = pf_ptr;
}

particle_filter_t* particle_filter_configuration_strategy_t::get_particle_filter() const
{
    return m_particle_filter;
}
