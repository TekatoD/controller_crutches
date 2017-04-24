/**
 *  @autor arssivka
 *  @date 4/24/17
 */

#include "Speech.h"
#include <sys/wait.h>
#include <sys/types.h>

Robot::Speech* Robot::Speech::GetInstance() {
    static Speech instance;
    return &instance;
}

bool Robot::Speech::Started() const noexcept {
    return m_ChildPid >= 0;
}

bool Robot::Speech::Start() {
    if (m_ChildPid >= 0) return true;
    if (pipe(m_Pipe) == -1) {
        return false;
    }

    if ((m_ChildPid = fork()) < 0) {
        close(m_Pipe[PIPE_READ]);
        close(m_Pipe[PIPE_WRITE]);
        return false;
    }

    if (m_ChildPid == 0) {
        close(STDOUT_FILENO);
        close(m_Pipe[PIPE_WRITE]);
        dup2(m_Pipe[PIPE_READ], STDIN_FILENO);
        execl("/usr/bin/festival", "/usr/bin/festival", "--tts", nullptr);
        close(m_Pipe[PIPE_READ]);
        exit(0);
    } else {
        close(m_Pipe[PIPE_READ]);
    }
    return true;
}

void Robot::Speech::Stop() {
    if (m_ChildPid >= 0) {
        int status;
        int pid;
        close(PIPE_WRITE);
        while ((pid = wait(&status)) != -1) {
            if (pid == m_ChildPid) {
                break;
            }
        }
    }
}

void Robot::Speech::Say(const std::string& str) {
    if (m_ChildPid >= 0) {
        write(m_Pipe[PIPE_WRITE], str.data(), str.size());
    }
}

Robot::Speech::~Speech() {
    Stop();
}

Robot::Speech::Speech()
        : m_ChildPid{-1},
          m_Pipe{-1, -1}  {

}
