#ifndef _DATA_READER_H_
#define _DATA_READER_H_

#include <iostream>
#include <stdexcept>
#include <fstream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>

namespace Localization {
    
class DataReader {
public:
    using control_data = std::vector<Eigen::Vector3f>;
    using measurement_bundle = std::vector<Eigen::Vector3f>;
    using measurement_data = std::vector<measurement_bundle>;
    using world_data  = std::vector<Eigen::Vector3f>;
    
    DataReader(std::string, std::string);
    
    control_data getControlData() const { return controlData; }
    measurement_data getMeasurementData() const { return measurementData; }
    world_data getWorldData() const { return worldData; }
private:
    control_data controlData;
    measurement_data measurementData;
    world_data worldData;
};

}
#endif