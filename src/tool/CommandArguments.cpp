/*!
 *  \autor arssivka
 *  \date 10/25/17
 */
#include "tool/CommandArguments.h"

Robot::CommandArguments::CommandArguments(int argc, char** argv)
        : m_argc(argc), m_argv(argv) {}

int Robot::CommandArguments::GetArgc() const {
    return m_argc;
}

void Robot::CommandArguments::SetArgc(int argc) {
    m_argc = argc;
}

char** Robot::CommandArguments::GetArgv() const {
    return m_argv;
}

void Robot::CommandArguments::SetArgv(char** argv) {
    m_argv = argv;
}
