/*
 *   Action.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <string.h>
#include <memory>
#include <log/Logger.h>
#include "motion/MotionStatus.h"
#include "motion/modules/Action.h"

using namespace Robot;


Action::Action() {
    m_debug = true;
    m_action_file = 0;
    m_playing = false;
}


Action::~Action() {
    if (m_action_file != 0)
        fclose(m_action_file);
}


bool Action::VerifyChecksum(PAGE* pPage) {
    unsigned char checksum = 0x00;
    unsigned char* pt = (unsigned char*) pPage;

    for (unsigned int i = 0; i < sizeof(PAGE); i++) {
        checksum += *pt;
        pt++;
    }

    if (checksum != 0xff)
        return false;

    return true;
}


void Action::SetChecksum(PAGE* pPage) {
    unsigned char checksum = 0x00;
    unsigned char* pt = (unsigned char*) pPage;

    pPage->header.checksum = 0x00;

    for (unsigned int i = 0; i < sizeof(PAGE); i++) {
        checksum += *pt;
        pt++;
    }

    pPage->header.checksum = (unsigned char) (0xff - checksum);
}


void Action::ResetPage(PAGE* pPage) {
    unsigned char* pt = (unsigned char*) pPage;

    for (unsigned int i = 0; i < sizeof(PAGE); i++) {
        *pt = 0x00;
        pt++;
    }

    pPage->header.schedule = TIME_BASE_SCHEDULE; // default time base
    pPage->header.repeat = 1;
    pPage->header.speed = 32;
    pPage->header.accel = 32;

    for (int i = 0; i < JointData::NUMBER_OF_JOINTS; i++)
        pPage->header.slope[i] = 0x55;

    for (int i = 0; i < MAXNUM_STEP; i++) {
        for (int j = 0; j < 31; j++)
            pPage->step[i].position[j] = INVALID_BIT_MASK;

        pPage->step[i].pause = 0;
        pPage->step[i].time = 0;
    }

    SetChecksum(pPage);
}


void Action::Initialize() {
    m_playing = false;

    for (int id = JointData::ID_R_SHOULDER_PITCH; id < JointData::NUMBER_OF_JOINTS; id++)
        m_Joint.SetValue(id, MotionStatus::m_CurrentJoints.GetValue(id));
}


bool Action::LoadFile(char* filename) {
    FILE* action = fopen(filename, "r+b");

#ifdef WEBOTS
    // Olivier.Michel@cyberbotics.com added the following line to allow opening a readonly file located in the Webots installation directory.
  // This is mainly problematic on Windows
    if( action == 0 ) action = fopen( filename, "rb" );
#endif

    if (action == 0) {
        LOG_ERROR << "ACTION: Can not open Action file!";
        return false;
    }

    fseek(action, 0, SEEK_END);
    if (ftell(action) != (long) (sizeof(PAGE) * MAXNUM_PAGE)) {
        LOG_ERROR << "ACTION: It's not an Action file!";
        fclose(action);
        return false;
    }

    if (m_action_file != 0)
        fclose(m_action_file);

    m_action_file = action;
    return true;
}


bool Action::CreateFile(char* filename) {
    FILE* action = fopen(filename, "ab");
    if (action == 0) {
        LOG_ERROR << "ACTION: Can not create Action file!";
        return false;
    }

    PAGE page;
    ResetPage(&page);
    for (int i = 0; i < MAXNUM_PAGE; i++)
        fwrite(&page, 1, sizeof(PAGE), action);

    if (m_action_file != 0)
        fclose(m_action_file);

    m_action_file = action;
    return true;
}


bool Action::Start(int iPage) {
    if (iPage < 1 || iPage >= MAXNUM_PAGE) {
        LOG_ERROR << "ACTION: Can not play page.(" << iPage << " is invalid index)";
        return false;
    }

    PAGE page;
    if (!LoadPage(iPage, &page))
        return false;

    return Start(iPage, &page);
}


bool Action::Start(char* namePage) {
    int index;
    PAGE page;

    for (index = 1; index < MAXNUM_PAGE; index++) {
        if (LoadPage(index, &page) == false)
            return false;

        if (strcmp(namePage, (char*) page.header.name) == 0)
            break;
    }

    return Start(index, &page);
}


bool Action::Start(int index, PAGE* pPage) {
    if (m_playing) {
        LOG_WARNING << "ACTION: Can not play page " << index << ".(Now playing)";
        return false;
    }

    m_play_page = *pPage;

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


void Action::Stop() {
    if (m_debug) {
        LOG_DEBUG << "ACTION: Action was stopped";
    }
    m_stop_playing = true;
}


void Action::Brake() {
    if (m_debug) {
        LOG_DEBUG << "ACTION: Action was broke";
    }
    m_playing = false;
}


bool Action::IsRunning() {
    return m_playing;
}


bool Action::IsRunning(int* iPage, int* iStep) {
    if (iPage != 0)
        *iPage = m_index_playing_page;

    if (iStep != 0)
        *iStep = m_page_step_count - 1;

    return IsRunning();
}


bool Action::LoadPage(int index, PAGE* pPage) {
    long position = (long) (sizeof(PAGE) * index);

    if (fseek(m_action_file, position, SEEK_SET) != 0)
        return false;

    if (fread(pPage, 1, sizeof(PAGE), m_action_file) != sizeof(PAGE))
        return false;

    if (!VerifyChecksum(pPage))
        ResetPage(pPage);

    return true;
}


bool Action::SavePage(int index, PAGE* pPage) {
    long position = (long) (sizeof(PAGE) * index);

    if (!VerifyChecksum(pPage))
        SetChecksum(pPage);

    if (fseek(m_action_file, position, SEEK_SET) != 0)
        return false;

    if (fwrite(pPage, 1, sizeof(PAGE), m_action_file) != sizeof(PAGE))
        return false;

    return true;
}


void Action::Process() {
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
    static unsigned short wpStartAngle1024[JointData::NUMBER_OF_JOINTS]; // ������ ���� ����
    static unsigned short wpTargetAngle1024[JointData::NUMBER_OF_JOINTS]; // ������ ���� ����
    static short int ipMovingAngle1024[JointData::NUMBER_OF_JOINTS]; // �� ������ �Ÿ�
    static short int ipMainAngle1024[JointData::NUMBER_OF_JOINTS]; // ��� �������� ������ �Ÿ�
    static short int ipAccelAngle1024[JointData::NUMBER_OF_JOINTS]; // ���� �������� ������ �Ÿ�
    static short int ipMainSpeed1024[JointData::NUMBER_OF_JOINTS]; // ��ǥ ��ӵ�
    static short int ipLastOutSpeed1024[JointData::NUMBER_OF_JOINTS]; // �� �� ������ �ӵ�(����)
    static short int ipGoalSpeed1024[JointData::NUMBER_OF_JOINTS]; // ���Ͱ� ���� �� ��ǥ�ӵ�
    static unsigned char bpFinishType[JointData::NUMBER_OF_JOINTS]; // ���� ������ ������ ����
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

        for (bID = JointData::ID_R_SHOULDER_PITCH; bID < JointData::NUMBER_OF_JOINTS; bID++) {
            if (m_Joint.GetEnable(bID)) {
                wpTargetAngle1024[bID] = MotionStatus::m_CurrentJoints.GetValue(bID);
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
            for (bID = JointData::ID_R_SHOULDER_PITCH; bID < JointData::NUMBER_OF_JOINTS; bID++) {
                // ���� ����ϴ� ������ ���
                if (m_Joint.GetEnable(bID)) {
                    if (ipMovingAngle1024[bID] == 0)
                        m_Joint.SetValue(bID, wpStartAngle1024[bID]);
                    else {
                        if (bSection == PRE_SECTION) {
                            iSpeedN = (short) (
                                    ((long) (ipMainSpeed1024[bID] - ipLastOutSpeed1024[bID]) * wUnitTimeCount) /
                                    wUnitTimeNum);
                            ipGoalSpeed1024[bID] = ipLastOutSpeed1024[bID] + iSpeedN;
                            ipAccelAngle1024[bID] = (short) (
                                    (((long) (ipLastOutSpeed1024[bID] + (iSpeedN >> 1)) * wUnitTimeCount * 144) / 15)
                                            >> 9);

                            m_Joint.SetValue(bID, wpStartAngle1024[bID] + ipAccelAngle1024[bID]);
                        } else if (bSection == MAIN_SECTION) {
                            m_Joint.SetValue(bID, wpStartAngle1024[bID] +
                                                  (short int) (((long) (ipMainAngle1024[bID]) * wUnitTimeCount) /
                                                               wUnitTimeNum));
                            ipGoalSpeed1024[bID] = ipMainSpeed1024[bID];
                        } else // POST_SECTION
                        {
                            if (wUnitTimeCount == (wUnitTimeNum - 1)) {
                                // ���� ������ ������ ���̱����� �׳� ��ǥ ��ġ ���� ���
                                m_Joint.SetValue(bID, wpTargetAngle1024[bID]);
                            } else {
                                if (bpFinishType[bID] == ZERO_FINISH) {
                                    iSpeedN = (short int) (((long) (0 - ipLastOutSpeed1024[bID]) * wUnitTimeCount) /
                                                           wUnitTimeNum);
                                    ipGoalSpeed1024[bID] = ipLastOutSpeed1024[bID] + iSpeedN;
                                    m_Joint.SetValue(bID, wpStartAngle1024[bID] + (short) (
                                            (((long) (ipLastOutSpeed1024[bID] + (iSpeedN >> 1)) * wUnitTimeCount *
                                              144) / 15) >> 9));
                                } else // NONE_ZERO_FINISH
                                {
                                    // MAIN Section�� �����ϰ� �۵�-����
                                    // step���� ������� ���� � ������ �����ϴ� ��Ȳ�� �߻��� �� �����Ƿ� �̷��� �� ���ۿ� ����
                                    m_Joint.SetValue(bID, wpStartAngle1024[bID] + (short int) (
                                            ((long) (ipMainAngle1024[bID]) * wUnitTimeCount) / wUnitTimeNum));
                                    ipGoalSpeed1024[bID] = ipMainSpeed1024[bID];
                                }
                            }
                        }
                    }

                    // lastest MX28 firmwares do not support compliance slopes
                    //m_Joint.SetSlope(bID, 1 << (m_PlayPage.header.slope[bID]>>4), 1 << (m_PlayPage.header.slope[bID]&0x0f));                    
                    m_Joint.SetPGain(bID, (256 >> (m_play_page.header.slope[bID] >> 4)) << 2);
                }
            }
        }
    } else if (wUnitTimeCount >= wUnitTimeNum) // ���� Section�� �Ϸ�Ǿ��ٸ�
    {
        wUnitTimeCount = 0;

        for (bID = JointData::ID_R_SHOULDER_PITCH; bID < JointData::NUMBER_OF_JOINTS; bID++) {
            if (m_Joint.GetEnable(bID)) {
                wpStartAngle1024[bID] = m_Joint.GetValue(bID);
                ipLastOutSpeed1024[bID] = ipGoalSpeed1024[bID];
            }
        }

        // Section ������Ʈ ( PRE -> MAIN -> POST -> (PAUSE or PRE) ... )
        if (bSection == PRE_SECTION) {
            // MAIN Section �غ�
            bSection = MAIN_SECTION;
            wUnitTimeNum = wUnitTimeTotalNum - (wAccelStep << 1);

            for (bID = JointData::ID_R_SHOULDER_PITCH; bID < JointData::NUMBER_OF_JOINTS; bID++) {
                if (m_Joint.GetEnable(bID)) {
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

            for (bID = JointData::ID_R_SHOULDER_PITCH; bID < JointData::NUMBER_OF_JOINTS; bID++) {
                if (m_Joint.GetEnable(bID))
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

            for (bID = JointData::ID_R_SHOULDER_PITCH; bID < JointData::NUMBER_OF_JOINTS; bID++) {
                if (m_Joint.GetEnable(bID))
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
                        LoadPage(wNextPlayPage, &m_next_play_page);
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
            for (bID = JointData::ID_R_SHOULDER_PITCH; bID < JointData::NUMBER_OF_JOINTS; bID++) {
                if (m_Joint.GetEnable(bID)) {
                    // ����, ����, �̷��� �������� ������ ���
                    ipAccelAngle1024[bID] = 0;

                    // Find current target angle
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

                    // Find Next target angle
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

            for (bID = JointData::ID_R_SHOULDER_PITCH; bID < JointData::NUMBER_OF_JOINTS; bID++) {
                if (m_Joint.GetEnable(bID)) {
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

bool Action::GetDebug() const {
    return m_debug;
}

void Action::SetDebug(bool debug) {
    m_debug = debug;
}

Action* Action::GetInstance() {
    static Action action;
    return &action;
}
