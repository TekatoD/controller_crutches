/**
 *  @autor arssivka
 *  @date 4/24/17
 */

#pragma once


#include <unistd.h>
#include <string>

namespace Robot {
    class Speech {
        enum {
            PIPE_WRITE = 0,
            PIPE_READ = 1
        };

    public:
        static Speech* GetInstance();

        Speech(const Speech&) = delete;

        Speech(Speech&&) = delete;

        Speech& operator=(const Speech&) = delete;

        Speech& operator=(Speech&&) = delete;

        bool Started() const noexcept;

        bool Start();

        void Stop();

        void Say(const std::string& str);

        ~Speech();

    private:
        Speech();

    private:
        int m_Pipe[2];
        pid_t m_ChildPid;
    };
}


