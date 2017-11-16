//
// Created by akovalev on 16.11.17.
//

#pragma once

#include <localization/particle_filter_t.h>
#include <config/configuration_strategy_t.h>

namespace drwn {

    class particle_filter_configuration_strategy_t : public configuration_strategy_t {
    public:
        static constexpr char DEFAULT_SECTION[] = "particle_filter";

        explicit particle_filter_configuration_strategy_t(drwn::particle_filter_t* pf_ptr = nullptr, std::string section = DEFAULT_SECTION);

        void read_config(const boost::property_tree::ptree& prop) override;
        void write_config(boost::property_tree::ptree& prop) const override;

        void set_particle_filter(drwn::particle_filter_t* pf_ptr);
        drwn::particle_filter_t* get_particle_filter() const;

    private:
        drwn::particle_filter_t* m_particle_filter{nullptr};
    };

}


