/*!
 *  \autor arssivka
 *  \date 10/25/17
 */
#include "config/CommandArguments.h"

drwn::CommandArguments::CommandArguments(int argc, char** argv)
        : m_argc(argc), m_argv(argv) {}

int drwn::CommandArguments::GetArgc() const {
    return m_argc;
}

void drwn::CommandArguments::SetArgc(int argc) {
    m_argc = argc;
}

char** drwn::CommandArguments::GetArgv() const {
    return m_argv;
}

void drwn::CommandArguments::SetArgv(char** argv) {
    m_argv = argv;
}
