//
// Created by akovalev on 16.11.17.
//

#include <localization/localization_t.h>

using namespace drwn;

localization_t* localization_t::get_instance() {
    static localization_t instance;
    return &instance;
}

localization_t::localization_t()
{

}

void localization_t::set_pose_shift(pose_2D_t pose_shift)
{

}

void localization_t::set_lines(std::vector<cv::Vec4i> lines)
{
//
}

void localization_t::update()
{

}

void localization_t::set_current_pose(pose_2D_t current_pose)
{
    assert(m_particle_filter != nullptr);
}

void localization_t::set_field(field_map_t field)
{
    assert(m_particle_filter != nullptr);
    m_particle_filter->set_field_map(field);
}

field_map_t localization_t::get_field() const
{
    assert(m_particle_filter != nullptr);
    return *(m_particle_filter->get_field_map());
}

void localization_t::set_camera_parameters(vision_utils::camera_parameters_t camera_params)
{

}

vision_utils::camera_parameters_t localization_t::get_camera_parameters()
{
    return m_camera_params;
}

void localization_t::set_pose_approximate_area(pose_2D_t min_pose, pose_2D_t max_pose)
{

}

void localization_t::set_pose_approximate_area_to_field()
{

}

void localization_t::set_particle_filter(particle_filter_t* pf_ptr)
{
    assert(pf_ptr != nullptr);
    m_particle_filter = pf_ptr;
}

particle_filter_t* localization_t::get_particle_filter() const
{
    assert(m_particle_filter != nullptr);
    return m_particle_filter;
}

pose_2D_t localization_t::get_calculated_pose_mean()
{
    assert(m_particle_filter != nullptr);
    return m_particle_filter->get_pose_mean();
}

pose_2D_t localization_t::get_calculated_pose_std_dev()
{
    assert(m_particle_filter != nullptr);
    return m_particle_filter->get_pose_std_dev();
}

bool localization_t::is_debug_enabled() const {
    return m_debug;
}

void localization_t::enable_debug(bool debug) {
    assert(m_particle_filter != nullptr);
    m_debug = debug;
    m_particle_filter->enable_debug(debug);
}


