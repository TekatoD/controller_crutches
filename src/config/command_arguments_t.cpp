/*!
 *  \autor arssivka
 *  \date 10/25/17
 */
#include "config/command_arguments_t.h"

drwn::command_arguments_t::command_arguments_t(int argc, char** argv)
        : m_argc(argc), m_argv(argv) {}

int drwn::command_arguments_t::get_argc() const {
    return m_argc;
}

void drwn::command_arguments_t::set_argc(int argc) {
    m_argc = argc;
}

char** drwn::command_arguments_t::get_argv() const {
    return m_argv;
}

void drwn::command_arguments_t::set_argv(char** argv) {
    m_argv = argv;
}
