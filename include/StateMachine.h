/*
 * StatusCheck.h
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#ifndef STATUSCHECK_H_
#define STATUSCHECK_H_

#include "CM730.h"
#include "minIni.h"
#include "Pose2D.h"

#define CNTRL_SECTION "controller"

namespace Robot {
    enum {
        BTN_START = 1,
        BTN_STOP = 2
    };

    enum {
        ROLE_UNKNOWN = 0,
        ROLE_SOCCER = 1,
        ROLE_GOALKEEPER = 2
    };

    class StateMachine {
    private:
        static int m_old_btn;
        static int m_is_started;
        static int m_role;

        static Pose2D m_spawn_pos;
        static Pose2D m_starting_pos;

    public:
        static int IsStarted();

        static const Pose2D &SpawnPosition();

        static const Pose2D &StartingPosition();

        static void Check(CM730& cm730);

        static void LoadINISettings(minIni* ini);

        static void LoadINISettings(minIni* ini, const std::string& section);

        static void SaveINISettings(minIni* ini);

        static void SaveINISettings(minIni* ini, const std::string& section);
    };
}

#endif /* STATUSCHECK_H_ */
