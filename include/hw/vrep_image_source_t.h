/**
 *  @autor tekatod
 *  @date 11/3/17
 */
#pragma once

#include "image_source_t.h"

namespace drwn {
    class vrep_image_source_t : public image_source_t {
    public:

        explicit vrep_image_source_t(std::string sensor_name = "camera", int width = 320, int height = 240);

        void connect();

        int get_client_id() const;

        void set_client_id(int client_id = -1);

        cv::Mat capture_frame() const override;

    private:
        bool m_debug{false};
    public:
        bool is_debug_enabled() const;

        void enable_debug(bool debug);

    private:
        std::string m_sensor_name{"camera"};
        int m_width{320};
        int m_height{240};
        mutable int m_resolution[2]{320, 240}; //Cause we need to change this while capturing frame in the const method
        int m_client_id{-1};
        mutable unsigned char* m_binary_image; //Cause we need to change this while capturing frame in the const method
        int m_sensor_handle{-1};
    };

}