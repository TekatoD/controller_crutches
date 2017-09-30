#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include "VrepCM730.h"

#define MAX_EXT_API_CONNECTIONS 255
#define NON_MATLAB_PARSING
extern "C" {
#include "extApi.h"
#include "extApiPlatform.h"
}



Robot::VrepCM730::VrepCM730(std::string device_postfix) : m_client_id(-1),
                                                                         m_device_postfix(device_postfix),
                                                                         m_connected(false) {
    for (int i = 0; i < ID_BROADCAST; i++) {
        m_BulkReadData[i] = BulkReadData();
        m_BulkReadData[i].length = MX28::MAXNUM_ADDRESS; //TODO: Dunno if it worth to do something more clever with length
    }
}

void Robot::VrepCM730::init_devices() {

    //initialize sensors readings
    simxFloat h;
    simxGetFloatSignal(m_client_id, (std::string("accelerometerX") + m_device_postfix).c_str(), &h, simx_opmode_streaming);
    simxGetFloatSignal(m_client_id, (std::string("accelerometerY") + m_device_postfix).c_str(), &h, simx_opmode_streaming);
    simxGetFloatSignal(m_client_id, (std::string("accelerometerZ") + m_device_postfix).c_str(), &h, simx_opmode_streaming);
    simxGetFloatSignal(m_client_id, (std::string("gyroX") + m_device_postfix).c_str(), &h, simx_opmode_streaming);
    simxGetFloatSignal(m_client_id, (std::string("gyroY") + m_device_postfix).c_str(), &h, simx_opmode_streaming);
    simxGetFloatSignal(m_client_id, (std::string("gyroZ") + m_device_postfix).c_str(), &h, simx_opmode_streaming);

    //get joints handlers
    m_joints[JointData::ID_R_SHOULDER_PITCH - 1] = this->connect_device("j_shoulder_pitch_r" + m_device_postfix);
    m_joints[JointData::ID_L_SHOULDER_PITCH - 1] = this->connect_device("j_shoulder_pitch_l" + m_device_postfix);
    m_joints[JointData::ID_R_SHOULDER_ROLL - 1] = this->connect_device("j_shoulder_roll_r" + m_device_postfix);
    m_joints[JointData::ID_L_SHOULDER_ROLL - 1] = this->connect_device("j_shoulder_roll_l" + m_device_postfix);
    m_joints[JointData::ID_R_ELBOW - 1] = this->connect_device("j_elbow_pitch_r" + m_device_postfix);
    m_joints[JointData::ID_L_ELBOW - 1] = this->connect_device("j_elbow_pitch_l" + m_device_postfix);
    m_joints[JointData::ID_R_HIP_YAW - 1] = this->connect_device("j_hip_yaw_r" + m_device_postfix);
    m_joints[JointData::ID_L_HIP_YAW - 1] = this->connect_device("j_hip_yaw_l" + m_device_postfix);
    m_joints[JointData::ID_R_HIP_ROLL - 1] = this->connect_device("j_hip_roll_r" + m_device_postfix);
    m_joints[JointData::ID_L_HIP_ROLL - 1] = this->connect_device("j_hip_roll_l" + m_device_postfix);
    m_joints[JointData::ID_R_HIP_PITCH - 1] = this->connect_device("j_hip_pitch_r" + m_device_postfix);
    m_joints[JointData::ID_L_HIP_PITCH - 1] = this->connect_device("j_hip_pitch_l" + m_device_postfix);
    m_joints[JointData::ID_R_KNEE - 1] = this->connect_device("j_knee_pitch_r" + m_device_postfix);
    m_joints[JointData::ID_L_KNEE - 1] = this->connect_device("j_knee_pitch_l" + m_device_postfix);
    m_joints[JointData::ID_R_ANKLE_PITCH - 1] = this->connect_device("j_ankle_pitch_r" + m_device_postfix);
    m_joints[JointData::ID_L_ANKLE_PITCH - 1] = this->connect_device("j_ankle_pitch_l" + m_device_postfix);
    m_joints[JointData::ID_R_ANKLE_ROLL - 1] = this->connect_device("j_ankle_roll_r" + m_device_postfix);
    m_joints[JointData::ID_L_ANKLE_ROLL - 1] = this->connect_device("j_ankle_roll_l" + m_device_postfix);
    m_joints[JointData::ID_HEAD_PAN - 1] = this->connect_device("j_head_yaw" + m_device_postfix);
    m_joints[JointData::ID_HEAD_TILT - 1] = this->connect_device("j_head_pitch" + m_device_postfix);
    std::cout << "\033[32mDevices initialized\033[0m" << std::endl;
}

void Robot::VrepCM730::SetClientId(int client_id) {
    m_client_id = client_id;
}

int Robot::VrepCM730::GetClientId() {
    return m_client_id;
}

int Robot::VrepCM730::connect_device(std::string device_name) {
    int object_handler;
    if(simxGetObjectHandle(m_client_id, device_name.c_str(),
                           &object_handler, simx_opmode_oneshot_wait) == simx_return_ok) {
        return object_handler;
    }
    throw std::runtime_error("Can't connect with " + device_name + " device");
}

int Robot::VrepCM730::SyncWrite(int start_addr, int each_length, int number, int *pParam) {
    simxPauseCommunication(m_client_id, 1);
//    this->DumpJoints("/home/tekatod/develop/Walking.txt", start_addr, each_length, number, pParam);
    for(size_t i = 0; i < number * each_length; i += each_length) {
        simxSetObjectIntParameter(m_client_id, m_joints[pParam[i] - 1], 2000, 1, simx_opmode_oneshot);
        simxSetObjectIntParameter(m_client_id, m_joints[pParam[i] - 1], 2001, 1, simx_opmode_oneshot);
//        //TODO: Check PID control (order: D I P)
//        simxSetObjectIntParameter(m_client_id, m_sim_devices[pParam[i]], 2004, (pParam[i + 1]  * M_PI / MX28::MAX_VALUE) * 4 / 1000, simx_opmode_oneshot);
//        simxSetObjectIntParameter(m_client_id, m_sim_devices[pParam[i]], 2003, (pParam[i + 2]  * M_PI / MX28::MAX_VALUE) * 1000 / 2048, simx_opmode_oneshot);
//        simxSetObjectIntParameter(m_client_id, m_sim_devices[pParam[i]], 2002, (pParam[i + 3]  * M_PI / MX28::MAX_VALUE) / 8, simx_opmode_oneshot);

        simxSetJointTargetPosition(m_client_id, m_joints[pParam[i] - 1],
                                   ((Robot::MX28::Value2Angle(CM730::MakeWord(pParam[i + 5], pParam[i + 6])) * M_PI) / 180),
                                   simx_opmode_oneshot);
    }
    simxPauseCommunication(m_client_id, 0);
    simxSynchronousTrigger(m_client_id);
}

//This method can be used in the SyncWrite method above for saving the sequence of servos values into the file.
void Robot::VrepCM730::DumpJoints(std::string file_name, int start_addr, int each_length, int number, int *pParam) {
    std::ofstream f(file_name, std::ios_base::app);
    for(size_t i = 0; i < number * each_length; i += each_length) {
        f << pParam[i] << " " << (Robot::MX28::Value2Angle(CM730::MakeWord(pParam[i + 5], pParam[i + 6])) * M_PI) / 180 << std::endl;
    }
    f << "END" << std::endl;
    f.close();
}

int Robot::VrepCM730::ReadWord(int id, int address, int* pValue, int* error) {
        auto get_sensor_data = [this, &error](std::string signal) {
            simxFloat data;
            *error = simxGetFloatSignal(m_client_id, signal.c_str(), &data, simx_opmode_buffer);
            if (*error != simx_return_ok) {
                data = 0;
            }
            return data;
        };
        auto norm_accel = [](double value) {
            return (int)((value + 39.24) / (78.48) * 1023);
        };
        auto norm_gyro = [](double value) {
            return (int)((value + 500) / (1000) * 1023);
        };
        switch(address) {
            case P_GYRO_Z_L:
                *pValue = norm_gyro(get_sensor_data("gyroZ" + m_device_postfix));
                break;
            case P_GYRO_Y_L:
                *pValue = norm_gyro(-get_sensor_data("gyroY" + m_device_postfix));
                break;
            case P_GYRO_X_L:
                *pValue = norm_gyro(get_sensor_data("gyroX" + m_device_postfix));
                break;
            case P_ACCEL_Z_L:
                *pValue = norm_accel(get_sensor_data("accelerometerZ" + m_device_postfix));
                break;
            case P_ACCEL_Y_L:
                *pValue = norm_accel(-get_sensor_data("accelerometerY" + m_device_postfix));
                break;
            case P_ACCEL_X_L:
                *pValue = norm_accel(get_sensor_data("accelerometerX" + m_device_postfix));
                break;
            default:
                simxFloat pos;

                simxSetObjectIntParameter(m_client_id, m_joints[id - 1], 2000, 1, simx_opmode_oneshot);
                simxSetObjectIntParameter(m_client_id, m_joints[id - 1], 2001, 1, simx_opmode_oneshot);
                *error = simxGetJointPosition(m_client_id, m_joints[id - 1], &pos, simx_opmode_oneshot);
                if (*error != simx_return_ok) {
                    pos = 0;
                }
                pos = (180 * pos) / M_PI;
//                std::cout << "Joint " << id << " " << pos << std::endl;
                *pValue = MX28::Angle2Value(pos);
                return SUCCESS;
    }
}

int Robot::VrepCM730::BulkRead() {
    for(size_t i = JointData::ID_R_SHOULDER_PITCH; i < JointData::NUMBER_OF_JOINTS; ++i) {
        int value;
        int error;
        this->ReadWord(i, 0, &value, &error);
        m_BulkReadData[i].table[MX28::P_PRESENT_POSITION_L] = CM730::GetLowByte(value);
        m_BulkReadData[i].table[MX28::P_PRESENT_POSITION_H] = CM730::GetHighByte(value);
    }
    m_BulkReadData->error = 0;  //TODO:: crutch

    for(size_t i = P_GYRO_Z_L; i < P_VOLTAGE; i += 2) { //TODO:: Check the loop
        int value;
        int error;
        this->ReadWord(0, i, &value, &error);
        m_BulkReadData[CM730::ID_CM].table[i] = CM730::GetLowByte(value);
        m_BulkReadData[CM730::ID_CM].table[i + 1] = CM730::GetHighByte(value);
        if(error == simx_return_ok) {
            m_BulkReadData->error = 0;
        }
    }
    m_BulkReadData[ID_CM].error = 0; //TODO:: crutch
}

int Robot::VrepCM730::WriteByte(int address, int value, int* error) {
    return SUCCESS;
}

int Robot::VrepCM730::WriteWord(int id, int address, int value, int* error) {
    if(address == MX28::P_PRESENT_POSITION_L) {
        simxSetObjectIntParameter(m_client_id, m_joints[id - 1], 2000, 1, simx_opmode_oneshot);
        simxSetObjectIntParameter(m_client_id, m_joints[id - 1], 2001, 1, simx_opmode_oneshot);
        *error = simxSetJointTargetPosition(m_client_id, m_joints[id - 1],
                                   (Robot::MX28::Value2Angle(value) * M_PI) / 180,
                                   simx_opmode_oneshot);
        return SUCCESS;
    }
    return SUCCESS;
}

int Robot::VrepCM730::ReadByte(int id, int address, int *pValue, int* error) {
    if(address == MX28::P_VERSION) {
        *pValue = 28;
    }
    return SUCCESS;
}

bool Robot::VrepCM730::Connect() {
    if(!m_connected && m_client_id != -1) {
        this->init_devices();
        this->BulkRead();
        m_connected = true;
    }
    return m_connected;
}

bool Robot::VrepCM730::DXLPowerOn() {
    return true;
}

Robot::VrepCM730::~VrepCM730() { }

bool Robot::VrepCM730::ChangeBaud(int baud) {
    return true;
}

void Robot::VrepCM730::Disconnect() { }

bool Robot::VrepCM730::MX28InitAll() {
    return true;
}

int Robot::VrepCM730::WriteWord(int address, int value, int *error) {
    return 0;
}

int Robot::VrepCM730::Ping(int id, int *error) {
    int p;
    *error = simxGetPingTime(m_client_id, &p);
    return p;
}

int Robot::VrepCM730::ReadTable(int id, int start_addr, int end_addr, unsigned char *table, int *error) {
    return 0;
}

int Robot::VrepCM730::WriteByte(int id, int address, int value, int *error) {
    return 0;
}

int Robot::VrepCM730::WriteTable(int id, int start_addr, int end_addr, unsigned char *table, int *error) {
    return 0;
}

void Robot::VrepCM730::MakeBulkReadPacket() { }
