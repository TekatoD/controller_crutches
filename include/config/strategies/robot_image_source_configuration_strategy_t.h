/// \autor arssivka
/// \date 11/7/17

#pragma once


#include <config/configuration_strategy_t.h>
#include <hw/robot_image_source_t.h>

namespace drwn {
    class robot_image_source_configuration_strategy_t
            : public configuration_strategy_t {
    public:
        static constexpr char DEFAULT_SECTION[] = "Robot Camera";

        robot_image_source_configuration_strategy_t(std::string section = DEFAULT_SECTION);

        void read_config(const boost::property_tree::ptree& prop) override;

        void write_config(boost::property_tree::ptree& prop) const override;

        robot_image_source_t* get_image_source() const noexcept;

        void set_image_source(robot_image_source_t* image_source) noexcept;

    private:
        robot_image_source_t* m_image_source{nullptr};
    };
}



