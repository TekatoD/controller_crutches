#include <localization/DataReader.h>

using namespace std;
using namespace Localization;
using namespace Eigen;

DataReader::DataReader(std::string sensorFilename, std::string worldFilename)
{
    ifstream sensorFile(sensorFilename);
    ifstream worldFile(worldFilename);
    
    if (!sensorFile.is_open() || !worldFile.is_open()) {
        throw runtime_error{"DataReader: Sensor data and world data files are required"}; 
    }
    
    string line;
    string delim = " ";
    
    measurement_bundle measBundle;
    while (getline(sensorFile, line)) {
        vector<string> tokens;
        string token;  
        string dataType;
        
        size_t pos = 0;
        while ((pos = line.find(delim)) != string::npos) {
            token = line.substr(0, pos);    
            if (token == "ODOMETRY" || token == "SENSOR") {
                dataType = token;
            } else {
                tokens.push_back(token);
            }
            line.erase(0, pos + delim.length());
        }
        tokens.push_back(line);
        
        if (dataType == "ODOMETRY") {
            if (controlData.size()) { 
                measurementData.push_back(measBundle);
                measBundle.clear();
            }
            
            // rot1 (float), trans (float), rot2 (float)
            Vector3f odom(3);
            for (size_t i = 0; i < tokens.size(); ++i) {
                odom(i) = stof(tokens[i]);
            }
            
            
            controlData.push_back(odom);
        } else {
            // landmark_id (int), range (float), bearing (float)
            Vector3f sens(3);
            for (size_t i = 0; i < tokens.size(); ++i) {
                sens(i) = stof(tokens[i]);
            }
            
            measBundle.push_back(sens);
        }
    }
    measurementData.push_back(measBundle);
    
    while (getline(worldFile, line)) {
        string token;  
        
        Vector3f landmarkData(3);
        size_t pos = 0;
        size_t i = 0;
        while ((pos = line.find(delim)) != string::npos) {
            token = line.substr(0, pos);    
            
            landmarkData(i++) = stof(token);
            
            line.erase(0, pos + delim.length());
        }
        landmarkData(i) = stof(line);
        
        worldData.push_back(landmarkData);
    }
    
    sensorFile.close();
    worldFile.close();
}