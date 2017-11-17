#include <iostream>
#include <stdexcept>
#include <string>
#include <eigen3/Eigen/Dense>

#include <minIni.h>
#include <pose_2D_t.h.h>
#include <math/Point.h>
#include <localization/particle_filter_t.h>
#include <localization/line_t.h>

using namespace Eigen;
using namespace localization;

#define INI_FILE_PATH "res/config.ini"

int main(int argc, char** argv)
{
    
    minIni ini(INI_FILE_PATH);
    field_map_t field;
    field.load_ini_settings(&ini);
    field.print_field_lines();
    
    std::vector< std::tuple<line_t, field_map_t::line_type_t>> tests {
        std::make_tuple(line_t(500.0f, 900.0f, 4000.0f, 900.0f), field_map_t::line_type_t::PENALTY_RIGHT_HEIGHT),
        std::make_tuple(line_t(500.0f, 0.0f, 500.0f, 3000.0f), field_map_t::line_type_t::FIELD_TOP),
    };
    
    for (auto& tuple : tests) {
        line_t intersectingLine = std::get<0>(tuple);
        field_map_t::line_type_t correctType = std::get<1>(tuple);
        
        field_map_t::line_type_t retType = field_map_t::line_type_t::NONE;
        Robot::Point2D isec;
        auto result = field.intersect_with_field(intersectingLine);
        
        retType = std::get<0>(result);
        isec = std::get<1>(result);
        
        std::cout << "=========" << std::endl;
        if (retType == correctType) {
            std::cout << "Correct" << std::endl;
            std::cout << "Intersected with line_type_t: " << (int)retType << std::endl;
            std::cout << "Intersection point: " << isec.X << ", " << isec.Y << std::endl;
        } else {
            std::cout << "Incorrect. Intersected with: " << (int)retType << ". Should be: " << (int)correctType << std::endl;
            std::cout << "Intersection point: " << isec.X << ", " << isec.Y << std::endl;
        }
    }
    
    /*
    line_t l1(0.0f, 0.0f, 10.0f, 10.0f);
    line_t l2(0.0f, 10.0f, 10.0f, 0.0f);
    Point2D intersection;
    
    // Should be (5, 5)
    if (line_t::IntersectLines(l1, l2, intersection)) {
        std::cout << intersection.X << ", " << intersection.Y << std::endl;
    }
    
    line_t l3(0.0, 5.0, 5.0, 5.0);
    line_t l4(10.0, 0.0, 10.0, 10.0);
    
    // Doesn't intersect. (line are collinear but don't intersect)
    if (line_t::intersect_lines(l3, l4, intersection)) {
        std::cout << intersection.X << ", " << intersection.Y << std::endl;
    }
    */
    
    /*
    if (argc != 3) {
        std::cout << "Need data" << std::endl;
        return -1;
    }
    
    try {
        localization::DataReader reader(argv[1], argv[2]);
        
        auto controlData = reader.getControlData();
        auto measurementData = reader.getMeasurementData();
        auto worldData = reader.getWorldData();
        
        Robot::Pose2D pose(0, 0, 0);
        localization::particle_filter_t pf(pose, worldData, 10);
        
        int max_timestep = controlData.size();
        Eigen::Vector3f noise = {0.01, 0.05, 0.01};
        for (int t = 0; t < max_timestep; t++) {
            auto& command = controlData[t];
            auto& measurementBundle = measurementData[t];
            
            pf.predict(command, noise);
            pf.correct(measurementBundle, noise);
            pf.resample();
        }
        
        auto particles = pf.get_particles();
        for (const auto& particle : particles) {
            std::cout << particle.pose << " | " << particle.weight << std::endl;
        }
        
        std::cout << "Pose mean: " << pf.get_pose_mean() << std::endl;
        std::cout << "Pose covariance: " << pf.getPoseCovariance() << std::endl;
        
        auto particle = pf.get_top_particle();
        std::cout << "Top particle pose: " << particle.pose << std::endl;
        std::cout << "Top particle weight: " << particle.weight << std::endl;
        
    } catch (const std::runtime_error& e) {
        std::cout << e.what() << std::endl;
    }
    */
    
    return 0;
}