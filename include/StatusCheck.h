/*
 * StatusCheck.h
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#ifndef STATUSCHECK_H_
#define STATUSCHECK_H_

#include "CM730.h"

namespace Robot {
    enum {
        BTN_START = 1,
        BTN_STOP = 2
    };

    class StatusCheck {
    private:
        static int m_old_btn;

    public:
        static int m_is_started;

        static void Check(CM730& cm730);
    };
}

#endif /* STATUSCHECK_H_ */
