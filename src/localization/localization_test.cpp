#include <iostream>
#include <stdexcept>
#include <string>
#include <eigen3/Eigen/Dense>

#include <localization/data_reader.h>

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
        
        std::cout << "Control data: " << controlData.size() << std::endl;
        std::cout << "Measurement data: " << measurementData.size() << std::endl;
        std::cout << "World data: " << worldData.size() << std::endl;
        
        // Inspect data
        int step = 50;
        std::cout << "Showing data at step = " << step << std::endl;
        std::cout << "Control data: " << std::endl;
        auto control = controlData[step];
        std::cout << control << std::endl;
        
        std::cout << "Measurements: " << std::endl;
        for (const auto& meas : measurementData[step]) {
            std::cout << "===============" << std::endl;
            std::cout << meas << std::endl;
            std::cout << "===============" << std::endl;
        }
    } catch (const std::runtime_error& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}