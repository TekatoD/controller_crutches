/*!
 *  \autor arssivka
 *  \date 10/25/17
 */
#pragma once


namespace drwn {
    class CommandArguments {
    public:
        CommandArguments() = default;

        CommandArguments(int argc, char** argv);

        int GetArgc() const;

        void SetArgc(int argc);

        char** GetArgv() const;

        void SetArgv(char** argv);

    private:
        int m_argc{0};
        char** m_argv{nullptr};
    };
}



