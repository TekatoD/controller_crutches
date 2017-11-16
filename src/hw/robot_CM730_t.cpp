/**
 *  @autor tekatod
 *  @date 9/20/17
 */
/*
 *   CM730_t.cpp
 *   This class is the secondary controler that controls the motors MX28
 *   Author: ROBOTIS
 *
 */
#include <stdio.h>
#include "hw/FSR_t.h"
#include "hw/robot_CM730_t.h"
#include "motion/motion_status_t.h"
#include "motion/kinematics_t.h"

using namespace drwn;


#define ID                    (2)
#define LENGTH                (3)
#define INSTRUCTION            (4)
#define ERRBIT                (4)
#define PARAMETER            (5)
#define DEFAULT_BAUDNUMBER    (1)

#define INST_PING            (1)
#define INST_READ            (2)
#define INST_WRITE            (3)
#define INST_REG_WRITE        (4)
#define INST_ACTION            (5)
#define INST_RESET            (6)
#define INST_SYNC_WRITE        (131)   // 0x83
#define INST_BULK_READ      (146)   // 0x92



robot_CM730_t::robot_CM730_t(platform_CM730_t* platform) {
    m_platform = platform;
    DEBUG_PRINT = true;
    m_bulk_read_tx_packet[LENGTH] = 0;
    for (int i = 0; i < ID_BROADCAST; i++)
        m_bulk_read_data[i] = bulk_read_data_t();
}


robot_CM730_t::~robot_CM730_t() {
    disconnect();
}


int robot_CM730_t::TxRx_packet(unsigned char* txpacket, unsigned char* rxpacket, int priority) {
    if (priority > 1)
        m_platform->low_priority_wait();
    if (priority > 0)
        m_platform->mid_priority_wait();
    m_platform->high_priority_wait();

    int res = TX_FAIL;
    int length = txpacket[LENGTH] + 4;

    txpacket[0] = 0xFF;
    txpacket[1] = 0xFF;
    txpacket[length - 1] = calculate_checksum(txpacket);

    if (DEBUG_PRINT == true) {
        fprintf(stderr, "\nTX: ");
        for (int n = 0; n < length; n++)
            fprintf(stderr, "%.2X ", txpacket[n]);

        fprintf(stderr, "INST: ");
        switch (txpacket[INSTRUCTION]) {
            case INST_PING:
                fprintf(stderr, "PING\n");
                break;

            case INST_READ:
                fprintf(stderr, "READ\n");
                break;

            case INST_WRITE:
                fprintf(stderr, "WRITE\n");
                break;

            case INST_REG_WRITE:
                fprintf(stderr, "REG_WRITE\n");
                break;

            case INST_ACTION:
                fprintf(stderr, "ACTION\n");
                break;

            case INST_RESET:
                fprintf(stderr, "RESET\n");
                break;

            case INST_SYNC_WRITE:
                fprintf(stderr, "SYNC_WRITE\n");
                break;

            case INST_BULK_READ:
                fprintf(stderr, "BULK_READ\n");
                break;

            default:
                fprintf(stderr, "UNKNOWN\n");
                break;
        }
    }

    if (length < (MAXNUM_TXPARAM + 6)) {
        m_platform->clear_port();
        if (m_platform->write_port(txpacket, length) == length) {
            if (txpacket[ID] != ID_BROADCAST) {
                int to_length = 0;

                if (txpacket[INSTRUCTION] == INST_READ)
                    to_length = txpacket[PARAMETER + 1] + 6;
                else
                    to_length = 6;

                m_platform->set_packet_timeout(length);

                int get_length = 0;
                if (DEBUG_PRINT == true)
                    fprintf(stderr, "RX: ");

                while (1) {
                    length = m_platform->read_port(&rxpacket[get_length], to_length - get_length);
                    if (DEBUG_PRINT == true) {
                        for (int n = 0; n < length; n++)
                            fprintf(stderr, "%.2X ", rxpacket[get_length + n]);
                    }
                    get_length += length;

                    if (get_length == to_length) {
                        // Find packet header
                        int i;
                        for (i = 0; i < (get_length - 1); i++) {
                            if (rxpacket[i] == 0xFF && rxpacket[i + 1] == 0xFF)
                                break;
                            else if (i == (get_length - 2) && rxpacket[get_length - 1] == 0xFF)
                                break;
                        }

                        if (i == 0) {
                            // Check checksum
                            unsigned char checksum = calculate_checksum(rxpacket);
                            if (DEBUG_PRINT == true)
                                fprintf(stderr, "CHK:%.2X\n", checksum);

                            if (rxpacket[get_length - 1] == checksum)
                                res = SUCCESS;
                            else
                                res = RX_CORRUPT;

                            break;
                        } else {
                            for (int j = 0; j < (get_length - i); j++)
                                rxpacket[j] = rxpacket[j + i];
                            get_length -= i;
                        }
                    } else {
                        if (m_platform->is_packet_timeout() == true) {
                            if (get_length == 0)
                                res = RX_TIMEOUT;
                            else
                                res = RX_CORRUPT;

                            break;
                        }
                    }
                }
            } else if (txpacket[INSTRUCTION] == INST_BULK_READ) {
                int to_length = 0;
                int num = (txpacket[LENGTH] - 3) / 3;

                for (int x = 0; x < num; x++) {
                    int _id = txpacket[PARAMETER + (3 * x) + 2];
                    int _len = txpacket[PARAMETER + (3 * x) + 1];
                    int _addr = txpacket[PARAMETER + (3 * x) + 3];

                    to_length += _len + 6;
                    m_bulk_read_data[_id].length = _len;
                    m_bulk_read_data[_id].start_address = _addr;
                }

                m_platform->set_packet_timeout(to_length * 1.5);

                int get_length = 0;
                if (DEBUG_PRINT == true)
                    fprintf(stderr, "RX: ");

                while (1) {
                    length = m_platform->read_port(&rxpacket[get_length], to_length - get_length);
                    if (DEBUG_PRINT == true) {
                        for (int n = 0; n < length; n++)
                            fprintf(stderr, "%.2X ", rxpacket[get_length + n]);
                    }
                    get_length += length;

                    if (get_length == to_length) {
                        res = SUCCESS;
                        break;
                    } else {
                        if (m_platform->is_packet_timeout() == true) {
                            if (get_length == 0)
                                res = RX_TIMEOUT;
                            else
                                res = RX_CORRUPT;

                            break;
                        }
                    }
                }

                for (int x = 0; x < num; x++) {
                    int _id = txpacket[PARAMETER + (3 * x) + 2];
                    m_bulk_read_data[_id].error = -1;
                }

                while (1) {
                    int i;
                    for (i = 0; i < get_length - 1; i++) {
                        if (rxpacket[i] == 0xFF && rxpacket[i + 1] == 0xFF)
                            break;
                        else if (i == (get_length - 2) && rxpacket[get_length - 1] == 0xFF)
                            break;
                    }

                    if (i == 0) {
                        // Check checksum
                        unsigned char checksum = calculate_checksum(rxpacket);
                        if (DEBUG_PRINT == true)
                            fprintf(stderr, "CHK:%.2X\n", checksum);

                        if (rxpacket[LENGTH + rxpacket[LENGTH]] == checksum) {
                            for (int j = 0; j < (rxpacket[LENGTH] - 2); j++)
                                m_bulk_read_data[rxpacket[ID]].table[m_bulk_read_data[rxpacket[ID]].start_address +
                                                                   j] = rxpacket[PARAMETER + j];

                            m_bulk_read_data[rxpacket[ID]].error = (int) rxpacket[ERRBIT];

                            int cur_packet_length = LENGTH + 1 + rxpacket[LENGTH];
                            to_length = get_length - cur_packet_length;
                            for (int j = 0; j <= to_length; j++)
                                rxpacket[j] = rxpacket[j + cur_packet_length];

                            get_length = to_length;
                            num--;
                        } else {
                            res = RX_CORRUPT;

                            for (int j = 0; j <= get_length - 2; j++)
                                rxpacket[j] = rxpacket[j + 2];

                            to_length = get_length -= 2;
                        }

                        if (num == 0)
                            break;
                        else if (get_length <= 6) {
                            if (num != 0) res = RX_CORRUPT;
                            break;
                        }

                    } else {
                        for (int j = 0; j < (get_length - i); j++)
                            rxpacket[j] = rxpacket[j + i];
                        get_length -= i;
                    }
                }
            } else
                res = SUCCESS;
        } else
            res = TX_FAIL;
    } else
        res = TX_CORRUPT;

    if (DEBUG_PRINT == true) {
        fprintf(stderr, "Time:%.2fms  ", m_platform->get_packet_time());
        fprintf(stderr, "RETURN: ");
        switch (res) {
            case SUCCESS:
                fprintf(stderr, "SUCCESS\n");
                break;

            case TX_CORRUPT:
                fprintf(stderr, "TX_CORRUPT\n");
                break;

            case TX_FAIL:
                fprintf(stderr, "TX_FAIL\n");
                break;

            case RX_FAIL:
                fprintf(stderr, "RX_FAIL\n");
                break;

            case RX_TIMEOUT:
                fprintf(stderr, "RX_TIMEOUT\n");
                break;

            case RX_CORRUPT:
                fprintf(stderr, "RX_CORRUPT\n");
                break;

            default:
                fprintf(stderr, "UNKNOWN\n");
                break;
        }
    }

    m_platform->high_priority_release();
    if (priority > 0)
        m_platform->mid_priority_release();
    if (priority > 1)
        m_platform->low_priority_release();

    return res;
}


unsigned char robot_CM730_t::calculate_checksum(unsigned char* packet) {
    unsigned char checksum = 0x00;
    for (int i = 2; i < packet[LENGTH] + 3; i++)
        checksum += packet[i];
    return (~checksum);
}


void robot_CM730_t::make_bulk_read_packet() {
    int number = 0;

    m_bulk_read_tx_packet[ID] = (unsigned char) ID_BROADCAST;
    m_bulk_read_tx_packet[INSTRUCTION] = INST_BULK_READ;
    m_bulk_read_tx_packet[PARAMETER] = (unsigned char) 0x0;

    if (ping(CM730_t::ID_CM, 0) == SUCCESS) {
        m_bulk_read_tx_packet[PARAMETER + 3 * number + 1] = 30;
        m_bulk_read_tx_packet[PARAMETER + 3 * number + 2] = CM730_t::ID_CM;
        m_bulk_read_tx_packet[PARAMETER + 3 * number + 3] = CM730_t::P_DXL_POWER;
        number++;
    }

//    for(int id = 1; id < joint_data_t::NUMBER_OF_JOINTS; id++)
//    {
//        if(motion_status_t::m_current_joints.get_enable(id))
//        {
//            m_bulk_read_tx_packet[PARAMETER+3*number+1] = 2;   // length
//            m_bulk_read_tx_packet[PARAMETER+3*number+2] = id;  // id
//            m_bulk_read_tx_packet[PARAMETER+3*number+3] = MX28_t::P_PRESENT_POSITION_L; // start address
//            number++;
//        }
//    }

    if (ping(FSR_t::ID_L_FSR, 0) == SUCCESS) {
        m_bulk_read_tx_packet[PARAMETER + 3 * number + 1] = 10;               // length
        m_bulk_read_tx_packet[PARAMETER + 3 * number + 2] = FSR_t::ID_L_FSR;   // id
        m_bulk_read_tx_packet[PARAMETER + 3 * number + 3] = FSR_t::P_FSR1_L;    // start address
        number++;
    }

    if (ping(FSR_t::ID_R_FSR, 0) == SUCCESS) {
        m_bulk_read_tx_packet[PARAMETER + 3 * number + 1] = 10;               // length
        m_bulk_read_tx_packet[PARAMETER + 3 * number + 2] = FSR_t::ID_R_FSR;   // id
        m_bulk_read_tx_packet[PARAMETER + 3 * number + 3] = FSR_t::P_FSR1_L;    // start address
        number++;
    }

    m_bulk_read_tx_packet[LENGTH] = (number * 3) + 3;
}


int robot_CM730_t::bulk_read() {
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0,};

    if (m_bulk_read_tx_packet[LENGTH] != 0)
        return TxRx_packet(m_bulk_read_tx_packet, rxpacket, 0);
    else {
        make_bulk_read_packet();
        return TX_FAIL;
    }
}


int robot_CM730_t::sync_write(int start_addr, int each_length, int number, int* pParam) {
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0,};
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0,};
    int n;

    txpacket[ID] = (unsigned char) ID_BROADCAST;
    txpacket[INSTRUCTION] = INST_SYNC_WRITE;
    txpacket[PARAMETER] = (unsigned char) start_addr;
    txpacket[PARAMETER + 1] = (unsigned char) (each_length - 1);
    for (n = 0; n < (number * each_length); n++)
        txpacket[PARAMETER + 2 + n] = (unsigned char) pParam[n];
    txpacket[LENGTH] = n + 4;

    return TxRx_packet(txpacket, rxpacket, 0);
}


bool robot_CM730_t::connect() {
    if (m_platform->open_port() == false) {
        fprintf(stderr, "\n Fail to open port\n");
        fprintf(stderr, " CM-730 is used by another program or do not have root privileges.\n\n");
        return false;
    }

    return (DXL_power_on() && MX28_init_all());
}


bool robot_CM730_t::change_baud(int baud) {
    if (m_platform->set_baud(baud) == false) {
        fprintf(stderr, "\n Fail to change baudrate\n");
        return false;
    }

    return DXL_power_on();
}


bool robot_CM730_t::DXL_power_on() {
    if (write_byte(CM730_t::ID_CM, CM730_t::P_DXL_POWER, 1, 0) == CM730_t::SUCCESS) {
        if (DEBUG_PRINT == true)
            fprintf(stderr, " Succeed to change Dynamixel power!\n");

        write_word(CM730_t::ID_CM, CM730_t::P_LED_HEAD_L, make_color(255, 128, 0), 0);
	fprintf(stderr, "write word\n");
        m_platform->sleep(300); // about 300msec
	fprintf(stderr, "sleep\n");
    } else {
        if (DEBUG_PRINT == true)
            fprintf(stderr, " Fail to change Dynamixel power!\n");
        return false;
    }

    return true;
}


bool robot_CM730_t::MX28_init_all() {
    // no limits for R_SHOULDER_PITCH
    // no limits for L_SHOULDER_PITCH

    if (write_word(joint_data_t::ID_R_SHOULDER_ROLL, MX28_t::P_CW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CW_LIMIT_R_SHOULDER_ROLL), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of R_SHOULDER_ROLL!\n");
    if (write_word(joint_data_t::ID_R_SHOULDER_ROLL, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_R_SHOULDER_ROLL), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of R_SHOULDER_ROLL!\n");

    if (write_word(joint_data_t::ID_L_SHOULDER_ROLL, MX28_t::P_CW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CW_LIMIT_L_SHOULDER_ROLL), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of L_SHOULDER_ROLL!\n");
    if (write_word(joint_data_t::ID_L_SHOULDER_ROLL, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_L_SHOULDER_ROLL), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of L_SHOULDER_ROLL!\n");

    if (write_word(joint_data_t::ID_R_ELBOW, MX28_t::P_CW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CW_LIMIT_R_ELBOW),
                   0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of R_ELBOW!\n");
    if (write_word(joint_data_t::ID_R_ELBOW, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_R_ELBOW),
                   0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of R_ELBOW!\n");

    if (write_word(joint_data_t::ID_L_ELBOW, MX28_t::P_CW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CW_LIMIT_L_ELBOW),
                   0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of L_ELBOW!\n");
    if (write_word(joint_data_t::ID_L_ELBOW, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_L_ELBOW),
                   0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of L_ELBOW!\n");

    if (write_word(joint_data_t::ID_R_HIP_YAW, MX28_t::P_CW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CW_LIMIT_R_HIP_YAW),
                   0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of R_HIP_YAW!\n");
    if (write_word(joint_data_t::ID_R_HIP_YAW, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_R_HIP_YAW), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of R_HIP_YAW!\n");

    if (write_word(joint_data_t::ID_L_HIP_YAW, MX28_t::P_CW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CW_LIMIT_L_HIP_YAW),
                   0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of L_HIP_YAW!\n");
    if (write_word(joint_data_t::ID_L_HIP_YAW, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_L_HIP_YAW), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of L_HIP_YAW!\n");

    if (write_word(joint_data_t::ID_R_HIP_ROLL, MX28_t::P_CW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CW_LIMIT_R_HIP_ROLL), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of R_HIP_ROLL!\n");
    if (write_word(joint_data_t::ID_R_HIP_ROLL, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_R_HIP_ROLL), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of R_HIP_ROLL!\n");

    if (write_word(joint_data_t::ID_L_HIP_ROLL, MX28_t::P_CW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CW_LIMIT_L_HIP_ROLL), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of L_HIP_ROLL!\n");
    if (write_word(joint_data_t::ID_L_HIP_ROLL, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_L_HIP_ROLL), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of L_HIP_ROLL!\n");

    if (write_word(joint_data_t::ID_R_HIP_PITCH, MX28_t::P_CW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CW_LIMIT_R_HIP_PITCH), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of R_HIP_PITCH!\n");
    if (write_word(joint_data_t::ID_R_HIP_PITCH, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_R_HIP_PITCH), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of R_HIP_PITCH!\n");

    if (write_word(joint_data_t::ID_L_HIP_PITCH, MX28_t::P_CW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CW_LIMIT_L_HIP_PITCH), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of L_HIP_PITCH!\n");
    if (write_word(joint_data_t::ID_L_HIP_PITCH, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_L_HIP_PITCH), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of L_HIP_PITCH!\n");

    if (write_word(joint_data_t::ID_R_KNEE, MX28_t::P_CW_ANGLE_LIMIT_L, MX28_t::angle_2_value(kinematics_t::CW_LIMIT_R_KNEE), 0) !=
        CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of R_KNEE!\n");
    if (write_word(joint_data_t::ID_R_KNEE, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_R_KNEE),
                   0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of R_KNEE!\n");

    if (write_word(joint_data_t::ID_L_KNEE, MX28_t::P_CW_ANGLE_LIMIT_L, MX28_t::angle_2_value(kinematics_t::CW_LIMIT_L_KNEE), 0) !=
        CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of L_KNEE!\n");
    if (write_word(joint_data_t::ID_L_KNEE, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_L_KNEE),
                   0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of L_KNEE!\n");

    if (write_word(joint_data_t::ID_R_ANKLE_PITCH, MX28_t::P_CW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CW_LIMIT_R_ANKLE_PITCH), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of R_ANKLE_PITCH!\n");
    if (write_word(joint_data_t::ID_R_ANKLE_PITCH, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_R_ANKLE_PITCH), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of R_ANKLE_PITCH!\n");

    if (write_word(joint_data_t::ID_L_ANKLE_PITCH, MX28_t::P_CW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CW_LIMIT_L_ANKLE_PITCH), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of L_ANKLE_PITCH!\n");
    if (write_word(joint_data_t::ID_L_ANKLE_PITCH, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_L_ANKLE_PITCH), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of L_ANKLE_PITCH!\n");

    if (write_word(joint_data_t::ID_R_ANKLE_ROLL, MX28_t::P_CW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CW_LIMIT_R_ANKLE_ROLL), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of R_ANKLE_ROLL!\n");
    if (write_word(joint_data_t::ID_R_ANKLE_ROLL, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_R_ANKLE_ROLL), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of R_ANKLE_ROLL!\n");

    if (write_word(joint_data_t::ID_L_ANKLE_ROLL, MX28_t::P_CW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CW_LIMIT_L_ANKLE_ROLL), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of L_ANKLE_ROLL!\n");
    if (write_word(joint_data_t::ID_L_ANKLE_ROLL, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_L_ANKLE_ROLL), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of L_ANKLE_ROLL!\n");

    if (write_word(joint_data_t::ID_HEAD_PAN, MX28_t::P_CW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CW_LIMIT_HEAD_PAN),
                   0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of HEAD_PAN!\n");
    if (write_word(joint_data_t::ID_HEAD_PAN, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_HEAD_PAN),
                   0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of HEAD_PAN!\n");

    if (write_word(joint_data_t::ID_HEAD_TILT, MX28_t::P_CW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CW_LIMIT_HEAD_TILT),
                   0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CW limit of HEAD_TILT!\n");
    if (write_word(joint_data_t::ID_HEAD_TILT, MX28_t::P_CCW_ANGLE_LIMIT_L,
                   MX28_t::angle_2_value(kinematics_t::CCW_LIMIT_HEAD_TILT), 0) != CM730_t::SUCCESS)
        fprintf(stderr, " Fail to change CCW limit of HEAD_TILT!\n");

    return true;
}


void robot_CM730_t::disconnect() {
    // Make the Head LED to green
    //write_word(CM730_t::ID_CM, CM730_t::P_LED_HEAD_L, make_color(0, 255, 0), 0);
    unsigned char txpacket[] = {0xFF, 0xFF, 0xC8, 0x05, 0x03, 0x1A, 0xE0, 0x03, 0x32};
    m_platform->write_port(txpacket, 9);

    m_platform->close_port();
}


int robot_CM730_t::write_byte(int address, int value, int* error) {
    return write_byte(ID_CM, address, value, error);
}


int robot_CM730_t::write_word(int address, int value, int* error) {
    return write_word(ID_CM, address, value, error);
}


int robot_CM730_t::ping(int id, int* error) {
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0,};
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0,};
    int result;

    txpacket[ID] = (unsigned char) id;
    txpacket[INSTRUCTION] = INST_PING;
    txpacket[LENGTH] = 2;

    result = TxRx_packet(txpacket, rxpacket, 2);
    if (result == SUCCESS && txpacket[ID] != ID_BROADCAST) {
        if (error != 0)
            *error = (int) rxpacket[ERRBIT];
    }

    return result;
}


int robot_CM730_t::read_byte(int id, int address, int* pValue, int* error) {
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0,};
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0,};
    int result;

    txpacket[ID] = (unsigned char) id;
    txpacket[INSTRUCTION] = INST_READ;
    txpacket[PARAMETER] = (unsigned char) address;
    txpacket[PARAMETER + 1] = 1;
    txpacket[LENGTH] = 4;

    result = TxRx_packet(txpacket, rxpacket, 2);
    if (result == SUCCESS) {
        *pValue = (int) rxpacket[PARAMETER];
        if (error != 0)
            *error = (int) rxpacket[ERRBIT];
    }

    return result;
}


int robot_CM730_t::read_word(int id, int address, int* pValue, int* error) {
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0,};
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0,};
    int result;

    txpacket[ID] = (unsigned char) id;
    txpacket[INSTRUCTION] = INST_READ;
    txpacket[PARAMETER] = (unsigned char) address;
    txpacket[PARAMETER + 1] = 2;
    txpacket[LENGTH] = 4;

    result = TxRx_packet(txpacket, rxpacket, 2);
    if (result == SUCCESS) {
        *pValue = make_word((int) rxpacket[PARAMETER], (int) rxpacket[PARAMETER + 1]);

        if (error != 0)
            *error = (int) rxpacket[ERRBIT];
    }

    return result;
}


int robot_CM730_t::read_table(int id, int start_addr, int end_addr, unsigned char* table, int* error) {
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0,};
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0,};
    int result;
    int length = end_addr - start_addr + 1;

    txpacket[ID] = (unsigned char) id;
    txpacket[INSTRUCTION] = INST_READ;
    txpacket[PARAMETER] = (unsigned char) start_addr;
    txpacket[PARAMETER + 1] = (unsigned char) length;
    txpacket[LENGTH] = 4;

    result = TxRx_packet(txpacket, rxpacket, 1);
    if (result == SUCCESS) {
        for (int i = 0; i < length; i++)
            table[start_addr + i] = rxpacket[PARAMETER + i];

        if (error != 0)
            *error = (int) rxpacket[ERRBIT];
    }

    return result;
}


int robot_CM730_t::write_byte(int id, int address, int value, int* error) {
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0,};
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0,};
    int result;

    txpacket[ID] = (unsigned char) id;
    txpacket[INSTRUCTION] = INST_WRITE;
    txpacket[PARAMETER] = (unsigned char) address;
    txpacket[PARAMETER + 1] = (unsigned char) value;
    txpacket[LENGTH] = 4;

    result = TxRx_packet(txpacket, rxpacket, 2);
    if (result == SUCCESS && id != ID_BROADCAST) {
        if (error != 0)
            *error = (int) rxpacket[ERRBIT];
    }

    return result;
}


int robot_CM730_t::write_word(int id, int address, int value, int* error) {
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0,};
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0,};
    int result;

    txpacket[ID] = (unsigned char) id;
    txpacket[INSTRUCTION] = INST_WRITE;
    txpacket[PARAMETER] = (unsigned char) address;
    txpacket[PARAMETER + 1] = (unsigned char) get_low_byte(value);
    txpacket[PARAMETER + 2] = (unsigned char) get_high_byte(value);
    txpacket[LENGTH] = 5;

    result = TxRx_packet(txpacket, rxpacket, 2);
    if (result == SUCCESS && id != ID_BROADCAST) {
        if (error != 0)
            *error = (int) rxpacket[ERRBIT];
    }

    return result;
}


int robot_CM730_t::write_table(int id, int start_addr, int end_addr, unsigned char* table, int* error) {
    unsigned char txpacket[MAXNUM_TXPARAM + 10] = {0,};
    unsigned char rxpacket[MAXNUM_RXPARAM + 10] = {0,};
    int result;
    int length = end_addr - start_addr + 1;

    txpacket[ID] = (unsigned char) id;
    txpacket[INSTRUCTION] = INST_WRITE;
    txpacket[PARAMETER] = (unsigned char) start_addr;
    for (int i = 0; i < length; i++)
        txpacket[PARAMETER + i + 1] = table[start_addr + i];
    txpacket[LENGTH] = 3 + length;

    result = TxRx_packet(txpacket, rxpacket, 2);
    if (result == SUCCESS && id != ID_BROADCAST) {
        if (error != 0)
            *error = (int) rxpacket[ERRBIT];
    }

    return result;
}
