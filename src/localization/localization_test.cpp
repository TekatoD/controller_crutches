#include <iostream>
#include <stdexcept>
#include <string>
#include <eigen3/Eigen/Dense>

#include <Pose2D.h>
#include <localization/DataReader.h>
#include <localization/ParticleFilter.h>

using namespace Eigen;

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cout << "Need data" << std::endl;
        return -1;
    }
    
    try {
        Localization::DataReader reader(argv[1], argv[2]);
        
        auto controlData = reader.getControlData();
        auto measurementData = reader.getMeasurementData();
        auto worldData = reader.getWorldData();
        
        Robot::Pose2D pose(0, 0, 0);
        Localization::ParticleFilter pf(pose, worldData, 100);
        
        int max_timestep = controlData.size();
        Eigen::Vector3f noise = {0.01, 0.05, 0.01};
        for (int t = 0; t < max_timestep; t++) {
            auto& command = controlData[t];
            auto& measurementBundle = measurementData[t];
            
            pf.predict(command, noise);
            pf.correct(measurementBundle, noise);
            pf.resample();
        }
        
        auto particles = pf.getParticles();
        for (const auto& particle : particles) {
            std::cout << particle.pose << std::endl;
        }
        
    } catch (const std::runtime_error& e) {
        std::cout << e.what() << std::endl;
    }
    
    return 0;
}