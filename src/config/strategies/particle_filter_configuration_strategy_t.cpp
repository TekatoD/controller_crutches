//
// Created by akovalev on 16.11.17.
//

#include <config/strategies/particle_filter_configuration_strategy_t.h>

using namespace drwn;
using namespace drwn;

particle_filter_configuration_strategy_t::particle_filter_configuration_strategy_t(particle_filter_t* pf_ptr, std::string section)
: configuration_strategy_t(section), m_particle_filter(pf_ptr)
{

}

void particle_filter_configuration_strategy_t::read_config(const boost::property_tree::ptree& prop)
{
    if (m_particle_filter == nullptr) { return; }
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

    if (num_particles) { m_particle_filter->set_particle_number(num_particles.get()); }
    if (init_x) { m_particle_filter->set_init_x(init_x.get()); }
    if (init_y) { m_particle_filter->set_init_y(init_y.get()); }
    if (init_theta) { m_particle_filter->set_init_theta(init_theta.get()); }
    if (random_particles) { m_particle_filter->set_random_particles(random_particles.get()); }
    if (min_x) { m_particle_filter->set_min_x(min_x.get()); }
    if (min_y) { m_particle_filter->set_min_y(min_y.get()); }
    if (min_theta) { m_particle_filter->set_min_theta(min_theta.get()); }
    if (max_x) { m_particle_filter->set_max_x(max_x.get()); }
    if (max_y) { m_particle_filter->set_max_y(max_y.get()); }
    if (max_theta) { m_particle_filter->set_max_theta(max_theta.get()); }
}

void particle_filter_configuration_strategy_t::write_config(boost::property_tree::ptree& prop) const
{
    //TODO:
}

void particle_filter_configuration_strategy_t::set_particle_filter(particle_filter_t* pf_ptr)
{
    if (pf_ptr == nullptr) { return;}
    m_particle_filter = pf_ptr;
}

particle_filter_t* particle_filter_configuration_strategy_t::get_particle_filter() const
{
    return m_particle_filter;
}
