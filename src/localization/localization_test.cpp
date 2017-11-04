#include <iostream>
#include <stdexcept>
#include <string>
#include <eigen3/Eigen/Dense>

#include <minIni.h>
#include <Pose2D.h>
#include <math/Point.h>
#include <localization/DataReader.h>
#include <localization/ParticleFilter.h>
#include <localization/LocalizationUtil.h>

using namespace Eigen;
using namespace Localization;

#define INI_FILE_PATH "res/config.ini"

int main(int argc, char** argv)
{
    
    /*
    minIni ini(INI_FILE_PATH);
    FieldMap field;
    field.LoadIniSettings(&ini);
    field.PrintFieldLines();
    
    std::vector< std::tuple<Line, FieldMap::LineType>> tests {
        std::make_tuple(Line(500.0f, 900.0f, 4000.0f, 900.0f), FieldMap::LineType::PENALTY_RIGHT_HEIGHT),
        std::make_tuple(Line(500.0f, 0.0f, 500.0f, 3000.0f), FieldMap::LineType::FIELD_TOP),
    };
    
    for (auto& tuple : tests) {
        Line intersectingLine = std::get<0>(tuple);
        FieldMap::LineType correctType = std::get<1>(tuple);
        
        FieldMap::LineType retType = FieldMap::LineType::NONE;
        Robot::Point2D isec;
        auto result = field.IntersectWithField(intersectingLine);
        
        retType = std::get<0>(result);
        isec = std::get<1>(result);
        
        std::cout << "=========" << std::endl;
        if (retType == correctType) {
            std::cout << "Correct" << std::endl;
            std::cout << "Intersected with LineType: " << (int)retType << std::endl;
            std::cout << "Intersection point: " << isec.X << ", " << isec.Y << std::endl;
        } else {
            std::cout << "Incorrect. Intersected with: " << (int)retType << ". Should be: " << (int)correctType << std::endl;
            std::cout << "Intersection point: " << isec.X << ", " << isec.Y << std::endl;
        }
    }
    */
    
    /*
    Line l1(0.0f, 0.0f, 10.0f, 10.0f);
    Line l2(0.0f, 10.0f, 10.0f, 0.0f);
    Point2D intersection;
    
    // Should be (5, 5)
    if (Line::IntersectLines(l1, l2, intersection)) {
        std::cout << intersection.X << ", " << intersection.Y << std::endl;
    }
    
    Line l3(0.0, 5.0, 5.0, 5.0);
    Line l4(10.0, 0.0, 10.0, 10.0);
    
    // Doesn't intersect. (line are collinear but don't intersect)
    if (Line::IntersectLines(l3, l4, intersection)) {
        std::cout << intersection.X << ", " << intersection.Y << std::endl;
    }
    */
    
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
        Localization::ParticleFilter pf(pose, worldData, 10);
        
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
            std::cout << particle.pose << " | " << particle.weight << std::endl;
        }
        
        std::cout << "Pose mean: " << pf.getPoseMean() << std::endl;
        std::cout << "Pose covariance: " << pf.getPoseCovariance() << std::endl;
        
        auto particle = pf.getTopParticle();
        std::cout << "Top particle pose: " << particle.pose << std::endl;
        std::cout << "Top particle weight: " << particle.weight << std::endl;
        
    } catch (const std::runtime_error& e) {
        std::cout << e.what() << std::endl;
    }
    
    return 0;
}