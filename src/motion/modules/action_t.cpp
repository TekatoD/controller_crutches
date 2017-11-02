/*
 *   Action.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <string.h>
#include <memory>
#include <log/trivial_logger_t.h>
#include "motion/motion_status_t.h"
#include "motion/modules/action_t.h"

using namespace drwn;


action_t::action_t() {
    m_debug = true;
    m_action_file = 0;
    m_playing = false;
}


action_t::~action_t() {
    if (m_action_file != 0)
        fclose(m_action_file);
}


bool action_t::verify_checksum(PAGE* p_page) {
    unsigned char checksum = 0x00;
    unsigned char* pt = (unsigned char*) p_page;

    for (unsigned int i = 0; i < sizeof(PAGE); i++) {
        checksum += *pt;
        pt++;
    }

    if (checksum != 0xff)
        return false;

    return true;
}


void action_t::set_checksum(PAGE* p_page) {
    unsigned char checksum = 0x00;
    unsigned char* pt = (unsigned char*) p_page;

    p_page->header.checksum = 0x00;

    for (unsigned int i = 0; i < sizeof(PAGE); i++) {
        checksum += *pt;
        pt++;
    }

    p_page->header.checksum = (unsigned char) (0xff - checksum);
}


void action_t::reset_page(PAGE* p_page) {
    unsigned char* pt = (unsigned char*) p_page;

    for (unsigned int i = 0; i < sizeof(PAGE); i++) {
        *pt = 0x00;
        pt++;
    }

    p_page->header.schedule = TIME_BASE_SCHEDULE; // default time base
    p_page->header.repeat = 1;
    p_page->header.speed = 32;
    p_page->header.accel = 32;

    for (int i = 0; i < joint_data_t::NUMBER_OF_JOINTS; i++)
        p_page->header.slope[i] = 0x55;

    for (int i = 0; i < MAXNUM_STEP; i++) {
        for (int j = 0; j < 31; j++)
            p_page->step[i].position[j] = INVALID_BIT_MASK;

        p_page->step[i].pause = 0;
        p_page->step[i].time = 0;
    }

    set_checksum(p_page);
}


void action_t::initialize() {
    m_playing = false;

    for (int id = joint_data_t::ID_R_SHOULDER_PITCH; id < joint_data_t::NUMBER_OF_JOINTS; id++)
        joint.set_value(id, motion_status_t::m_current_joints.get_value(id));
}


void action_t::load_file(const char* filename) {
    FILE* action = fopen(filename, "r+b");

    if (action == nullptr) {
        throw std::runtime_error("Can not open action_t file!");
    }

    fseek(action, 0, SEEK_END);
    if (ftell(action) != (long) (sizeof(PAGE) * MAXNUM_PAGE)) {
        fclose(action);
        throw std::runtime_error("It's not an action_t file!");
    }

    if (m_action_file != nullptr)
        fclose(m_action_file);

    m_action_file = action;
}


bool action_t::create_file(char* filename) {
    FILE* action = fopen(filename, "ab");
    if (action == nullptr) {
        LOG_ERROR << "ACTION: Can not create Action file!";
        return false;
    }

    PAGE page{};
    reset_page(&page);
    for (int i = 0; i < MAXNUM_PAGE; i++)
        fwrite(&page, 1, sizeof(PAGE), action);

    if (m_action_file != nullptr)
        fclose(m_action_file);

    m_action_file = action;
    return true;
}


bool action_t::start(int i_page) {
    if (i_page < 1 || i_page >= MAXNUM_PAGE) {
        LOG_ERROR << "ACTION: Can not play page.(" << i_page << " is invalid index)";
        return false;
    }

    PAGE page{};
    if (!load_page(i_page, &page))
        return false;

    return start(i_page, &page);
}


bool action_t::start(char* name_page) {
    int index;
    PAGE page{};

    for (index = 1; index < MAXNUM_PAGE; index++) {
        if (!load_page(index, &page))
            return false;

        if (strcmp(name_page, (char*) page.header.name) == 0)
            break;
    }

    return start(index, &page);
}


bool action_t::start(int index, PAGE* p_page) {
    if (m_playing) {
        LOG_WARNING << "ACTION: Can not play page " << index << ".(Now playing)";
        return false;
    }

    m_play_page = *p_page;

    if (m_play_page.header.repeat == 0 || m_play_page.header.stepnum == 0) {
        LOG_WARNING << "ACTION: Page " << index << " has no action";
        return false;
    }

    if (m_debug) {
        LOG_DEBUG << "ACTION: Page " << index << " has been playing";
    }

    m_index_playing_page = index;
    m_first_driving_start = true;
    m_playing = true;
    return true;
}


void action_t::stop() {
    if (m_debug) {
        LOG_DEBUG << "ACTION: Action was stopped";
    }
    m_stop_playing = true;
}


void action_t::brake() {
    if (m_debug) {
        LOG_DEBUG << "ACTION: Action was broke";
    }
    m_playing = false;
}


bool action_t::is_running() {
    return m_playing;
}


bool action_t::is_running(int* i_page, int* i_step) {
    if (i_page != 0)
        *i_page = m_index_playing_page;

    if (i_step != 0)
        *i_step = m_page_step_count - 1;

    return is_running();
}


bool action_t::load_page(int index, PAGE* p_page) {
    long position = (long) (sizeof(PAGE) * index);

    if (fseek(m_action_file, position, SEEK_SET) != 0)
        return false;

    if (fread(p_page, 1, sizeof(PAGE), m_action_file) != sizeof(PAGE))
        return false;

    if (!verify_checksum(p_page))
        reset_page(p_page);

    return true;
}


bool action_t::save_page(int index, PAGE* p_page) {
    long position = (long) (sizeof(PAGE) * index);

    if (!verify_checksum(p_page))
        set_checksum(p_page);

    if (fseek(m_action_file, position, SEEK_SET) != 0)
        return false;

    return fwrite(p_page, 1, sizeof(PAGE), m_action_file) == sizeof(PAGE);
}


void action_t::process() {
    //////////////////// ���� ����
    unsigned char bID;
    unsigned long ulTotalTime256T;
    unsigned long ulPreSectionTime256T;
    unsigned long ulMainTime256T;
    long lStartSpeed1024_PreTime_256T;
    long lMovingAngle_Speed1024Scale_256T_2T;
    long lDivider1, lDivider2;
    unsigned short wMaxAngle1024;
    unsigned short wMaxSpeed256;
    unsigned short wTmp;
    unsigned short wPrevTargetAngle; // Start position
    unsigned short wCurrentTargetAngle; // Target position
    unsigned short wNextTargetAngle; // Next target position
    unsigned char bDirectionChanged;

    ///////////////// Static ����
    static unsigned short wpStartAngle1024[joint_data_t::NUMBER_OF_JOINTS]; // ������ ���� ����
    static unsigned short wpTargetAngle1024[joint_data_t::NUMBER_OF_JOINTS]; // ������ ���� ����
    static short int ipMovingAngle1024[joint_data_t::NUMBER_OF_JOINTS]; // �� ������ �Ÿ�
    static short int ipMainAngle1024[joint_data_t::NUMBER_OF_JOINTS]; // ��� �������� ������ �Ÿ�
    static short int ipAccelAngle1024[joint_data_t::NUMBER_OF_JOINTS]; // ���� �������� ������ �Ÿ�
    static short int ipMainSpeed1024[joint_data_t::NUMBER_OF_JOINTS]; // ��ǥ ��ӵ�
    static short int ipLastOutSpeed1024[joint_data_t::NUMBER_OF_JOINTS]; // �� �� ������ �ӵ�(����)
    static short int ipGoalSpeed1024[joint_data_t::NUMBER_OF_JOINTS]; // ���Ͱ� ���� �� ��ǥ�ӵ�
    static unsigned char bpFinishType[joint_data_t::NUMBER_OF_JOINTS]; // ���� ������ ������ ����
    short int iSpeedN;
    static unsigned short wUnitTimeCount;
    static unsigned short wUnitTimeNum;
    static unsigned short wPauseTime;
    static unsigned short wUnitTimeTotalNum;
    static unsigned short wAccelStep;
    static unsigned char bSection;
    static unsigned char bPlayRepeatCount;
    static unsigned short wNextPlayPage;

    /////////////// Enum ����

    /**************************************
    * Section             /----\
    *                    /|    |\
    *        /+---------/ |    | \
    *       / |        |  |    |  \
    * -----/  |        |  |    |   \----
    *      PRE  MAIN   PRE MAIN POST PAUSE
    ***************************************/
    enum {
        PRE_SECTION, MAIN_SECTION, POST_SECTION, PAUSE_SECTION
    };
    enum {
        ZERO_FINISH, NONE_ZERO_FINISH
    };

    if (!m_playing)
        return;

    if (m_first_driving_start) // ó�� �����Ҷ�
    {
        m_first_driving_start = false; //First Process end
        m_playing_finished = false;
        m_stop_playing = false;
        wUnitTimeCount = 0;
        wUnitTimeNum = 0;
        wPauseTime = 0;
        bSection = PAUSE_SECTION;
        m_page_step_count = 0;
        bPlayRepeatCount = m_play_page.header.repeat;
        wNextPlayPage = 0;

        for (bID = joint_data_t::ID_R_SHOULDER_PITCH; bID < joint_data_t::NUMBER_OF_JOINTS; bID++) {
            if (joint.get_enable(bID)) {
                wpTargetAngle1024[bID] = motion_status_t::m_current_joints.get_value(bID);
                ipLastOutSpeed1024[bID] = 0;
                ipMovingAngle1024[bID] = 0;
                ipGoalSpeed1024[bID] = 0;
            }
        }
    }

    if (wUnitTimeCount < wUnitTimeNum) // ���� �������̶��
    {
        wUnitTimeCount++;
        if (bSection == PAUSE_SECTION) {
        } else {
            for (bID = joint_data_t::ID_R_SHOULDER_PITCH; bID < joint_data_t::NUMBER_OF_JOINTS; bID++) {
                // ���� ����ϴ� ������ ���
                if (joint.get_enable(bID)) {
                    if (ipMovingAngle1024[bID] == 0)
                        joint.set_value(bID, wpStartAngle1024[bID]);
                    else {
                        if (bSection == PRE_SECTION) {
                            iSpeedN = (short) (
                                    ((long) (ipMainSpeed1024[bID] - ipLastOutSpeed1024[bID]) * wUnitTimeCount) /
                                    wUnitTimeNum);
                            ipGoalSpeed1024[bID] = ipLastOutSpeed1024[bID] + iSpeedN;
                            ipAccelAngle1024[bID] = (short) (
                                    (((long) (ipLastOutSpeed1024[bID] + (iSpeedN >> 1)) * wUnitTimeCount * 144) / 15)
                                            >> 9);

                            joint.set_value(bID, wpStartAngle1024[bID] + ipAccelAngle1024[bID]);
                        } else if (bSection == MAIN_SECTION) {
                            joint.set_value(bID, wpStartAngle1024[bID] +
                                                 (short int) (((long) (ipMainAngle1024[bID]) * wUnitTimeCount) /
                                                              wUnitTimeNum));
                            ipGoalSpeed1024[bID] = ipMainSpeed1024[bID];
                        } else // POST_SECTION
                        {
                            if (wUnitTimeCount == (wUnitTimeNum - 1)) {
                                // ���� ������ ������ ���̱����� �׳� ��ǥ ��ġ ���� ���
                                joint.set_value(bID, wpTargetAngle1024[bID]);
                            } else {
                                if (bpFinishType[bID] == ZERO_FINISH) {
                                    iSpeedN = (short int) (((long) (0 - ipLastOutSpeed1024[bID]) * wUnitTimeCount) /
                                                           wUnitTimeNum);
                                    ipGoalSpeed1024[bID] = ipLastOutSpeed1024[bID] + iSpeedN;
                                    joint.set_value(bID, wpStartAngle1024[bID] + (short) (
                                            (((long) (ipLastOutSpeed1024[bID] + (iSpeedN >> 1)) * wUnitTimeCount *
                                              144) / 15) >> 9));
                                } else // NONE_ZERO_FINISH
                                {
                                    // MAIN Section�� �����ϰ� �۵�-����
                                    // step���� ������� ���� � ������ �����ϴ� ��Ȳ�� �߻��� �� �����Ƿ� �̷��� �� ���ۿ� ����
                                    joint.set_value(bID, wpStartAngle1024[bID] + (short int) (
                                            ((long) (ipMainAngle1024[bID]) * wUnitTimeCount) / wUnitTimeNum));
                                    ipGoalSpeed1024[bID] = ipMainSpeed1024[bID];
                                }
                            }
                        }
                    }

                    // lastest MX28 firmwares do not support compliance slopes
                    //Joint.SetSlope(bID, 1 << (m_PlayPage.header.slope[bID]>>4), 1 << (m_PlayPage.header.slope[bID]&0x0f));
                    joint.set_p_gain(bID, (256 >> (m_play_page.header.slope[bID] >> 4)) << 2);
                }
            }
        }
    } else if (wUnitTimeCount >= wUnitTimeNum) // ���� Section�� �Ϸ�Ǿ��ٸ�
    {
        wUnitTimeCount = 0;

        for (bID = joint_data_t::ID_R_SHOULDER_PITCH; bID < joint_data_t::NUMBER_OF_JOINTS; bID++) {
            if (joint.get_enable(bID)) {
                wpStartAngle1024[bID] = joint.get_value(bID);
                ipLastOutSpeed1024[bID] = ipGoalSpeed1024[bID];
            }
        }

        // Section ������Ʈ ( PRE -> MAIN -> POST -> (PAUSE or PRE) ... )
        if (bSection == PRE_SECTION) {
            // MAIN Section �غ�
            bSection = MAIN_SECTION;
            wUnitTimeNum = wUnitTimeTotalNum - (wAccelStep << 1);

            for (bID = joint_data_t::ID_R_SHOULDER_PITCH; bID < joint_data_t::NUMBER_OF_JOINTS; bID++) {
                if (joint.get_enable(bID)) {
                    if (bpFinishType[bID] == NONE_ZERO_FINISH) {
                        if ((wUnitTimeTotalNum - wAccelStep) == 0) // ��� ������ ���� ���ٸ�
                            ipMainAngle1024[bID] = 0;
                        else
                            ipMainAngle1024[bID] = (short) (
                                    (((long) (ipMovingAngle1024[bID] - ipAccelAngle1024[bID])) * wUnitTimeNum) /
                                    (wUnitTimeTotalNum - wAccelStep));
                    } else // ZERO_FINISH
                        ipMainAngle1024[bID] = ipMovingAngle1024[bID] - ipAccelAngle1024[bID] -
                                               (short int) ((((long) ipMainSpeed1024[bID] * wAccelStep * 12) / 5) >> 8);
                }
            }
        } else if (bSection == MAIN_SECTION) {
            // POST Section �غ�
            bSection = POST_SECTION;
            wUnitTimeNum = wAccelStep;

            for (bID = joint_data_t::ID_R_SHOULDER_PITCH; bID < joint_data_t::NUMBER_OF_JOINTS; bID++) {
                if (joint.get_enable(bID))
                    ipMainAngle1024[bID] = ipMovingAngle1024[bID] - ipMainAngle1024[bID] - ipAccelAngle1024[bID];
            }
        } else if (bSection == POST_SECTION) {
            // Pause time ���������� �޶���
            if (wPauseTime) {
                bSection = PAUSE_SECTION;
                wUnitTimeNum = wPauseTime;
            } else {
                bSection = PRE_SECTION;
            }
        } else if (bSection == PAUSE_SECTION) {
            // PRE Section �غ�
            bSection = PRE_SECTION;

            for (bID = joint_data_t::ID_R_SHOULDER_PITCH; bID < joint_data_t::NUMBER_OF_JOINTS; bID++) {
                if (joint.get_enable(bID))
                    ipLastOutSpeed1024[bID] = 0;
            }
        }

        // PRE Section�ÿ� ��� �غ� �Ѵ�.
        if (bSection == PRE_SECTION) {
            if (m_playing_finished) // ����� �����ٸ�
            {
                if (m_debug && m_playing) {
                    LOG_DEBUG << "ACTION: Playing  was finished";
                }
                m_playing = false;
                return;
            }

            m_page_step_count++;

            if (m_page_step_count > m_play_page.header.stepnum) // ���� ������ ����� �����ٸ�
            {
                // ���� ������ ����
                m_play_page = m_next_play_page;
                if (m_index_playing_page != wNextPlayPage)
                    bPlayRepeatCount = m_play_page.header.repeat;
                m_page_step_count = 1;
                m_index_playing_page = wNextPlayPage;
            }

            if (m_page_step_count == m_play_page.header.stepnum) // ������ �����̶��
            {
                // ���� ������ �ε�
                if (m_stop_playing) // ��� ���� ����� �ִٸ�
                {
                    wNextPlayPage = m_play_page.header.exit; // ���� �������� Exit ��������
                } else {
                    bPlayRepeatCount--;
                    if (bPlayRepeatCount > 0) // �ݺ� Ƚ���� ���Ҵٸ�
                        wNextPlayPage = m_index_playing_page; // ���� �������� ���� ��������
                    else // �ݺ��� ���ߴٸ�
                        wNextPlayPage = m_play_page.header.next; // ���� �������� Next ��������
                }

                if (wNextPlayPage == 0) // ����� ���� �������� ���ٸ� ���� ���ܱ����ϰ� ����
                    m_playing_finished = true;
                else {
                    // ���������� �ε�(������ �޸� ����, �ٸ��� ���� �б�)
                    if (m_index_playing_page != wNextPlayPage)
                        load_page(wNextPlayPage, &m_next_play_page);
                    else
                        m_next_play_page = m_play_page;

                    // ����� ������ ���ٸ� ���� ���ܱ����ϰ� ����
                    if (m_next_play_page.header.repeat == 0 || m_next_play_page.header.stepnum == 0)
                        m_playing_finished = true;
                }
            }

            //////// Step �Ķ���� ���
            wPauseTime =
                    (((unsigned short) m_play_page.step[m_page_step_count - 1].pause) << 5) / m_play_page.header.speed;
            wMaxSpeed256 = ((unsigned short) m_play_page.step[m_page_step_count - 1].time *
                            (unsigned short) m_play_page.header.speed) >> 5;
            if (wMaxSpeed256 == 0)
                wMaxSpeed256 = 1;
            wMaxAngle1024 = 0;

            ////////// Joint�� �Ķ���� ���
            for (bID = joint_data_t::ID_R_SHOULDER_PITCH; bID < joint_data_t::NUMBER_OF_JOINTS; bID++) {
                if (joint.get_enable(bID)) {
                    // ����, ����, �̷��� �������� ������ ���
                    ipAccelAngle1024[bID] = 0;

                    // Find current m_target angle
                    if (m_play_page.step[m_page_step_count - 1].position[bID] & INVALID_BIT_MASK)
                        wCurrentTargetAngle = wpTargetAngle1024[bID];
                    else
                        wCurrentTargetAngle = m_play_page.step[m_page_step_count - 1].position[bID];

                    // Update start, prev_target, curr_target
                    wpStartAngle1024[bID] = wpTargetAngle1024[bID];
                    wPrevTargetAngle = wpTargetAngle1024[bID];
                    wpTargetAngle1024[bID] = wCurrentTargetAngle;

                    // Find Moving offset
                    ipMovingAngle1024[bID] = (int) (wpTargetAngle1024[bID] - wpStartAngle1024[bID]);

                    // Find Next m_target angle
                    if (m_page_step_count == m_play_page.header.stepnum) // ���� ������ �������̶��
                    {
                        if (m_playing_finished) // ���� �����̶��
                            wNextTargetAngle = wCurrentTargetAngle;
                        else {
                            if (m_next_play_page.step[0].position[bID] & INVALID_BIT_MASK)
                                wNextTargetAngle = wCurrentTargetAngle;
                            else
                                wNextTargetAngle = m_next_play_page.step[0].position[bID];
                        }
                    } else {
                        if (m_play_page.step[m_page_step_count].position[bID] & INVALID_BIT_MASK)
                            wNextTargetAngle = wCurrentTargetAngle;
                        else
                            wNextTargetAngle = m_play_page.step[m_page_step_count].position[bID];
                    }

                    // Find direction change
                    if (((wPrevTargetAngle < wCurrentTargetAngle) && (wCurrentTargetAngle < wNextTargetAngle))
                        || ((wPrevTargetAngle > wCurrentTargetAngle) && (wCurrentTargetAngle > wNextTargetAngle))) {
                        // ��� �����ϰų� �����ϰ�, Ȥ�� ���ٸ�(��, �ҿ��� ���� ���ٸ�)
                        bDirectionChanged = 0;
                    } else {
                        bDirectionChanged = 1;
                    }

                    // Find finish type
                    if (bDirectionChanged || wPauseTime || m_playing_finished) {
                        bpFinishType[bID] = ZERO_FINISH;
                    } else {
                        bpFinishType[bID] = NONE_ZERO_FINISH;
                    }

                    if (m_play_page.header.schedule == SPEED_BASE_SCHEDULE) {
                        //MaxAngle1024 update
                        if (ipMovingAngle1024[bID] < 0)
                            wTmp = -ipMovingAngle1024[bID];
                        else
                            wTmp = ipMovingAngle1024[bID];

                        if (wTmp > wMaxAngle1024)
                            wMaxAngle1024 = wTmp;
                    }
                }
            }

            //�ð��� ����ؼ� �ٽ� 7.8msec�� ������(<<7)-�׽ð����ȿ� 7.8msec�� ������� ����� ��
            //���� ��ȯ�ڿ� ��/�ӵ��� ���ϰ�(�ð�)�� �ð��� �ٽ� 7.8msec�� ����ִ��� ���
            //���� ��ȯ ---  �� :1024��->300����,  �ӵ�: 256�� ->720��
            //wUnitTimeNum = ((wMaxAngle1024*300/1024) /(wMaxSpeed256 * 720/256)) /7.8msec;
            //             = ((128*wMaxAngle1024*300/1024) /(wMaxSpeed256 * 720/256)) ;    (/7.8msec == *128)
            //             = (wMaxAngle1024*40) /(wMaxSpeed256 *3);
            if (m_play_page.header.schedule == TIME_BASE_SCHEDULE)
                wUnitTimeTotalNum = wMaxSpeed256; //TIME BASE 051025
            else
                wUnitTimeTotalNum = (wMaxAngle1024 * 40) / (wMaxSpeed256 * 3);

            wAccelStep = m_play_page.header.accel;
            if (wUnitTimeTotalNum <= (wAccelStep << 1)) {
                if (wUnitTimeTotalNum == 0)
                    wAccelStep = 0;
                else {
                    wAccelStep = (wUnitTimeTotalNum - 1) >> 1;
                    if (wAccelStep == 0)
                        wUnitTimeTotalNum = 0; //�����̷��� ��� ����,����� �� �����̻��� �����ؾ�
                }
            }

            ulTotalTime256T = ((unsigned long) wUnitTimeTotalNum) << 1;// /128 * 256
            ulPreSectionTime256T = ((unsigned long) wAccelStep) << 1;// /128 * 256
            ulMainTime256T = ulTotalTime256T - ulPreSectionTime256T;
            lDivider1 = ulPreSectionTime256T + (ulMainTime256T << 1);
            lDivider2 = (ulMainTime256T << 1);

            if (lDivider1 == 0)
                lDivider1 = 1;

            if (lDivider2 == 0)
                lDivider2 = 1;

            for (bID = joint_data_t::ID_R_SHOULDER_PITCH; bID < joint_data_t::NUMBER_OF_JOINTS; bID++) {
                if (joint.get_enable(bID)) {
                    lStartSpeed1024_PreTime_256T =
                            (long) ipLastOutSpeed1024[bID] * ulPreSectionTime256T; //  *300/1024 * 1024/720 * 256 * 2
                    lMovingAngle_Speed1024Scale_256T_2T = (((long) ipMovingAngle1024[bID]) * 2560L) / 12;

                    if (bpFinishType[bID] == ZERO_FINISH)
                        ipMainSpeed1024[bID] = (short int) (
                                (lMovingAngle_Speed1024Scale_256T_2T - lStartSpeed1024_PreTime_256T) / lDivider2);
                    else
                        ipMainSpeed1024[bID] = (short int) (
                                (lMovingAngle_Speed1024Scale_256T_2T - lStartSpeed1024_PreTime_256T) / lDivider1);

                    if (ipMainSpeed1024[bID] > 1023)
                        ipMainSpeed1024[bID] = 1023;

                    if (ipMainSpeed1024[bID] < -1023)
                        ipMainSpeed1024[bID] = -1023;
                }
            }

            wUnitTimeNum = wAccelStep; //PreSection
        }
    }
}

bool action_t::is_debug_enabled() const noexcept {
    return m_debug;
}

void action_t::enable_debug(bool debug) noexcept {
    m_debug = debug;
}

action_t* action_t::GetInstance() {
    static action_t action;
    return &action;
}
