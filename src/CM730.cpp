#define MAX_EXT_API_CONNECTIONS 255
#define NON_MATLAB_PARSING
extern "C" {
    #include "extApi.h"
    #include "extApi.c"
    #include "extApiPlatform.h"
    #include "extApiPlatform.c"
}

#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include "CM730.h"
#include "motion/JointData.h"

Robot::BulkReadData::BulkReadData()
        :
        start_address(0),
        length(MX28::MAXNUM_ADDRESS),
        error(-1) {
    for (int i = 0; i < MX28::MAXNUM_ADDRESS; i++)
        table[i] = 0;
}


int Robot::BulkReadData::ReadByte(int address) {
    if (address >= start_address && address < (start_address + length))
        return (int) table[address];

    return 0;
}


int Robot::BulkReadData::ReadWord(int address) {
    if (address >= start_address && address < (start_address + length))
        return CM730::MakeWord(table[address], table[address + 1]);

    return 0;
}

Robot::CM730::CM730(std::string server_ip, int server_port, int client_id, std::string device_postfix) {
    if(client_id =- -1) {
        m_client_id = simxStart((simxChar *) server_ip.c_str(), server_port, true, true, 5000, 5);
    }
    else {
        m_client_id = client_id;
    }
    if (m_client_id == -1) {
        std::cout << "Can't connect with sim" << std::endl;
    }
    simxStartSimulation(m_client_id, simx_opmode_oneshot);
    simxSynchronous(m_client_id, 1);
    m_device_postfix = device_postfix;
    init_devices();
    for (int i = 0; i < ID_BROADCAST; i++) {
        m_BulkReadData[i] = BulkReadData();
    }

    this->BulkRead();
}

Robot::CM730::CM730(int client_id, std::string device_postfix) {
    std::string server_ip("127.0.0.1");
    int server_port = 19997;
    if(client_id == -1) {
        m_client_id = simxStart((simxChar *) server_ip.c_str(), server_port, true, true, 5000, 5);
    } else {
        m_client_id = client_id;
    }
    if (m_client_id == -1) {
        std::cout << "Can't connect with sim" << std::endl;
    }
    simxSynchronous(m_client_id, 1);
    simxStartSimulation(m_client_id, simx_opmode_oneshot);
    m_device_postfix = device_postfix;
    init_devices();
    for (int i = 0; i < ID_BROADCAST; i++) {
        m_BulkReadData[i] = BulkReadData();
    }
    this->BulkRead();
}

void Robot::CM730::init_devices() {
//    const char* vrep_joint_names[] = {
//            "j_shoulder_pitch_r",
//            "j_shoulder_pitch_l",
//            "j_shoulder_roll_r",
//            "j_shoulder_roll_l",
//            "j_elbow_pitch_r",
//            "j_elbow_pitch_l",
//            "j_hip_yaw_r",
//            "j_hip_yaw_l",
//            "j_hip_roll_r",
//            "j_hip_roll_l",
//            "j_hip_pitch_r",
//            "j_hip_pitch_l",
//            "j_knee_pitch_r",
//            "j_knee_pitch_l",
//            "j_ankle_pitch_r",
//            "j_ankle_pitch_l",
//            "j_ankle_roll_r",
//            "j_ankle_roll_l",
//            "j_head_yaw",
//            "j_head_pitch"
//    };
    m_sim_devices.reserve(20);

    //initialize sensors readings
    simxFloat h;
    simxGetFloatSignal(m_client_id, (std::string("accelerometerX") + m_device_postfix).c_str(), &h, simx_opmode_streaming);
    simxGetFloatSignal(m_client_id, (std::string("accelerometerY") + m_device_postfix).c_str(), &h, simx_opmode_streaming);
    simxGetFloatSignal(m_client_id, (std::string("accelerometerZ") + m_device_postfix).c_str(), &h, simx_opmode_streaming);
    simxGetFloatSignal(m_client_id, (std::string("gyroX") + m_device_postfix).c_str(), &h, simx_opmode_streaming);
    simxGetFloatSignal(m_client_id, (std::string("gyroY") + m_device_postfix).c_str(), &h, simx_opmode_streaming);
    simxGetFloatSignal(m_client_id, (std::string("gyroZ") + m_device_postfix).c_str(), &h, simx_opmode_streaming);

    //get joints handlers
    m_sim_devices.emplace(std::make_pair(JointData::ID_R_SHOULDER_PITCH, this->connect_device("j_shoulder_pitch_r" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_L_SHOULDER_PITCH, this->connect_device("j_shoulder_pitch_l" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_R_SHOULDER_ROLL, this->connect_device("j_shoulder_roll_r" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_L_SHOULDER_ROLL, this->connect_device("j_shoulder_roll_l" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_R_ELBOW, this->connect_device("j_elbow_pitch_r" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_L_ELBOW, this->connect_device("j_elbow_pitch_l" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_R_HIP_YAW, this->connect_device("j_hip_yaw_r" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_L_HIP_YAW, this->connect_device("j_hip_yaw_l" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_R_HIP_ROLL, this->connect_device("j_hip_roll_r" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_L_HIP_ROLL, this->connect_device("j_hip_roll_l" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_R_HIP_PITCH, this->connect_device("j_hip_pitch_r" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_L_HIP_PITCH, this->connect_device("j_hip_pitch_l" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_R_KNEE, this->connect_device("j_knee_pitch_r" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_L_KNEE, this->connect_device("j_knee_pitch_l" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_R_ANKLE_PITCH, this->connect_device("j_ankle_pitch_r" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_L_ANKLE_PITCH, this->connect_device("j_ankle_pitch_l" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_R_ANKLE_ROLL, this->connect_device("j_ankle_roll_r" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_L_ANKLE_ROLL, this->connect_device("j_ankle_roll_l" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_HEAD_PAN, this->connect_device("j_head_yaw" + m_device_postfix)));
    m_sim_devices.emplace(std::make_pair(JointData::ID_HEAD_TILT, this->connect_device("j_head_pitch" + m_device_postfix)));
    std::cout << "Devices initialized" << std::endl;
}

int Robot::CM730::get_client_id() {
    return m_client_id;
}

int Robot::CM730::connect_device(std::string device_name) {
    int object_handler;
    if(simxGetObjectHandle(m_client_id, device_name.c_str(),
                           &object_handler, simx_opmode_oneshot_wait) == simx_return_ok) {
        return std::move(object_handler);
    }
    return -1;
}

int Robot::CM730::SyncWrite(int start_addr, int each_length, int number, int *pParam) {
    simxPauseCommunication(m_client_id, 1);
//    this->DumpJoints("/home/tekatod/develop/Walking.txt", start_addr, each_length, number, pParam);
    for(size_t i = 0; i < number * each_length; i += each_length) {
        simxSetObjectIntParameter(m_client_id, m_sim_devices[pParam[i]], 2000, 1, simx_opmode_oneshot);
        simxSetObjectIntParameter(m_client_id, m_sim_devices[pParam[i]], 2001, 1, simx_opmode_oneshot);
//        //TODO: Check PID control (order: D I P)
//        simxSetObjectIntParameter(m_client_id, m_sim_devices[pParam[i]], 2004, (pParam[i + 1]  * M_PI / MX28::MAX_VALUE) * 4 / 1000, simx_opmode_oneshot);
//        simxSetObjectIntParameter(m_client_id, m_sim_devices[pParam[i]], 2003, (pParam[i + 2]  * M_PI / MX28::MAX_VALUE) * 1000 / 2048, simx_opmode_oneshot);
//        simxSetObjectIntParameter(m_client_id, m_sim_devices[pParam[i]], 2002, (pParam[i + 3]  * M_PI / MX28::MAX_VALUE) / 8, simx_opmode_oneshot);

        simxSetJointTargetPosition(m_client_id, m_sim_devices[pParam[i]],
                                   ((Robot::MX28::Value2Angle(CM730::MakeWord(pParam[i + 5], pParam[i + 6])) * M_PI) / 180),
                                   simx_opmode_oneshot);
//        if (m_sim_devices[pParam[i]] == JointData::ID_L_ANKLE_ROLL) {
//            std::cout << "POSL!!!: " << Robot::MX28::Value2Angle(CM730::MakeWord(pParam[i + 5], pParam[i + 6])) << std::endl;
//        }
//        if (m_sim_devices[pParam[i]] == JointData::ID_R_ANKLE_ROLL) {
//            std::cout << "POSR!!!: " << Robot::MX28::Value2Angle(CM730::MakeWord(pParam[i + 5], pParam[i + 6])) << std::endl;
//        }
//
//        if (m_sim_devices[pParam[i]] == JointData::ID_L_SHOULDER_ROLL) {
//            std::cout << "POSSL!!!: " << Robot::MX28::Value2Angle(CM730::MakeWord(pParam[i + 5], pParam[i + 6])) << std::endl;
//        }
//
//        if (m_sim_devices[pParam[i]] == JointData::ID_R_SHOULDER_ROLL) {
//            std::cout << "POSSR!!!: " << Robot::MX28::Value2Angle(CM730::MakeWord(pParam[i + 5], pParam[i + 6])) << std::endl;
//        }

    }
    simxPauseCommunication(m_client_id, 0);
    simxSynchronousTrigger(m_client_id);
}

void Robot::CM730::DumpJoints(std::string file_name, int start_addr, int each_length, int number, int *pParam) {
    std::ofstream f(file_name, std::ios_base::app);
    for(size_t i = 0; i < number * each_length; i += each_length) {
        f << pParam[i] << " " << (Robot::MX28::Value2Angle(CM730::MakeWord(pParam[i + 5], pParam[i + 6])) * M_PI) / 180 << std::endl;
    }
    f << "END" << std::endl;
    f.close();
}

int Robot::CM730::ReadWord(int id, int address, int* pValue, int* error) {
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

                simxSetObjectIntParameter(m_client_id, m_sim_devices[id], 2000, 1, simx_opmode_oneshot);
                simxSetObjectIntParameter(m_client_id, m_sim_devices[id], 2001, 1, simx_opmode_oneshot);
                *error = simxGetJointPosition(m_client_id, m_sim_devices[id], &pos, simx_opmode_oneshot);
                if (*error != simx_return_ok) {
                    pos = 0;
                }
                pos = (180 * pos) / M_PI;
//                std::cout << "Joint " << id << " " << pos << std::endl;
                *pValue = MX28::Angle2Value(pos);
                return SUCCESS;
    }
}

int Robot::CM730::BulkRead() {
//    simxPauseCommunication(m_client_id, 1);
//    std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
    for(size_t i = JointData::ID_R_SHOULDER_PITCH; i < JointData::NUMBER_OF_JOINTS; ++i) {
        int value;
        int error;
        this->ReadWord(i, 0, &value, &error);
        m_BulkReadData[i].table[MX28::P_PRESENT_POSITION_L] = GetLowByte(value);
        m_BulkReadData[i].table[MX28::P_PRESENT_POSITION_H] = GetHighByte(value);
        if(error == simx_return_ok) {
            m_BulkReadData->error = 0;
        }

    }
//    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - tp);
//    std::cout << "Joints: " << ms.count() << std::endl;
//    tp = std::chrono::high_resolution_clock::now();
    for(size_t i = P_GYRO_Z_L; i < P_VOLTAGE; i += 2) { //TODO:: Check the loop
        int value;
        int error;
        this->ReadWord(0, i, &value, &error);
        m_BulkReadData[ID_CM].table[i] = GetLowByte(value);
        m_BulkReadData[ID_CM].table[i + 1] = GetHighByte(value);
        if(error == simx_return_ok) {
            m_BulkReadData->error = 0;
        }
    }
    m_BulkReadData[ID_CM].error = 0; //TODO:: crutchk
//    ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - tp);
//    std::cout << "Sensors: " << ms.count() << std::endl;

//    simxPauseCommunication(m_client_id, 0);
//    simxSynchronousTrigger(m_client_id);
}

int Robot::CM730::WriteByte(int address, int value, int* error) {
    return SUCCESS;
}

int Robot::CM730::WriteWord(int id, int address, int value, int* error) {
    if(address == MX28::P_PRESENT_POSITION_L) {
        simxSetObjectIntParameter(m_client_id, m_sim_devices[id], 2000, 1, simx_opmode_oneshot);
        simxSetObjectIntParameter(m_client_id, m_sim_devices[id], 2001, 1, simx_opmode_oneshot);
        *error = simxSetJointTargetPosition(m_client_id, m_sim_devices[id],
                                   (Robot::MX28::Value2Angle(value) * M_PI) / 180,
                                   simx_opmode_oneshot);
        return SUCCESS;
    }
    return SUCCESS;
}

int Robot::CM730::ReadByte(int id, int address, int *pValue, int* error) {
    if(address == MX28::P_VERSION) {
        *pValue = 28;
    }
    return SUCCESS;
}

bool Robot::CM730::Connect() {
    return (m_client_id != -1);
}

void Robot::CM730::DXLPowerOn() {}

int Robot::CM730::MakeWord(int lowbyte, int highbyte) {
    unsigned short word;

    word = highbyte;
    word = word << 8;
    word = word + lowbyte;

    return (int) word;
}


int Robot::CM730::GetLowByte(int word) {
    unsigned short temp;
    temp = word & 0xff;
    return (int) temp;
}


int Robot::CM730::GetHighByte(int word) {
    unsigned short temp;
    temp = word & 0xff00;
    return (int) (temp >> 8);
}

Robot::CM730::~CM730() {
    simxStopSimulation(m_client_id, simx_opmode_oneshot_wait);
    simxFinish(m_client_id);
}