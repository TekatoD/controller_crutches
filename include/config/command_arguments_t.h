/*!
 *  \autor arssivka
 *  \date 10/25/17
 */
#pragma once


namespace drwn {
    class command_arguments_t {
    public:
        command_arguments_t() = default;

        command_arguments_t(int argc, char** argv);

        int get_argc() const;

        void set_argc(int argc);

        char** get_argv() const;

        void set_argv(char** argv);

    private:
        int m_argc{0};
        char** m_argv{nullptr};
    };
}



