/*
 *   LinuxCM730.cpp
 *
 *   Author: ROBOTIS
 *
 */
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <log/Logger.h>
#include <iomanip>
#include "hw/LinuxCM730.h"

using namespace Robot;


LinuxCM730::LinuxCM730(const char* name) {
    sem_init(&m_LowSemID, 0, 1);
    sem_init(&m_MidSemID, 0, 1);
    sem_init(&m_HighSemID, 0, 1);
    SetPortName(name);
}

LinuxCM730::~LinuxCM730() {
    ClosePort();
}

void LinuxCM730::SetPortName(const char* name) {
    strcpy(m_port_name, name);
}

bool LinuxCM730::OpenPort() {
    struct termios newtio;
    struct serial_struct serinfo;
    double baudrate = 1000000.0; //bps (1Mbps)

    ClosePort();

    if (m_debug) LOG_DEBUG << "CM730: " << m_port_name << " open ";

    if ((m_socket_fd = open(m_port_name, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
        goto UART_OPEN_ERROR;

    if (m_debug) LOG_DEBUG << "CM730: success!";

    // You must set 38400bps!
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcsetattr(m_socket_fd, TCSANOW, &newtio);

    if (m_debug)
        LOG_DEBUG << std::setprecision(1) << "CM730: Set " << baudrate << "bps";

    // Set non-standard baudrate
    if (ioctl(m_socket_fd, TIOCGSERIAL, &serinfo) < 0)
        goto UART_OPEN_ERROR;

    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    serinfo.custom_divisor = serinfo.baud_base / baudrate;

    if (ioctl(m_socket_fd, TIOCSSERIAL, &serinfo) < 0) {
        if (m_debug) LOG_ERROR << "CM730: failed!";
        goto UART_OPEN_ERROR;
    }

    if (m_debug) LOG_DEBUG << "CM730: success!";

    tcflush(m_socket_fd, TCIFLUSH);

    m_byte_transfer_time = (1000.0 / baudrate) * 12.0;

    return true;

    UART_OPEN_ERROR:
    if (m_debug) LOG_ERROR << "CM730: failed!";
    ClosePort();
    return false;
}

bool LinuxCM730::SetBaud(int baud) {
    struct serial_struct serinfo;
    int baudrate = (int) (2000000.0f / (float) (baud + 1));

    if (m_socket_fd == -1)
        return false;

    if (ioctl(m_socket_fd, TIOCGSERIAL, &serinfo) < 0) {
        LOG_ERROR << "CM730: Cannot get serial info";
        return false;
    }

    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    serinfo.custom_divisor = serinfo.baud_base / baudrate;

    if (ioctl(m_socket_fd, TIOCSSERIAL, &serinfo) < 0) {
        LOG_ERROR << "CM730: Cannot set serial info";
        return false;
    }

    ClosePort();
    OpenPort();

    m_byte_transfer_time = (1000.0f / baudrate) * 12.0f * 8;

    return true;
}

void LinuxCM730::ClosePort() {
    if (m_socket_fd != -1)
        close(m_socket_fd);
    m_socket_fd = -1;
}

void LinuxCM730::ClearPort() {
    tcflush(m_socket_fd, TCIFLUSH);
}

int LinuxCM730::WritePort(unsigned char* packet, int numPacket) {
    return write(m_socket_fd, packet, numPacket);
}

int LinuxCM730::ReadPort(unsigned char* packet, int numPacket) {
    return read(m_socket_fd, packet, numPacket);
}

void sem_wait_nointr(sem_t* sem) {
    int sem_result, sem_count = 0;
    do {
        sem_result = sem_wait(sem);
    } while ((sem_result == -1) && (errno == EINTR));
}

void LinuxCM730::LowPriorityWait() {
    sem_wait_nointr(&m_LowSemID);
}

void LinuxCM730::MidPriorityWait() {
    sem_wait_nointr(&m_MidSemID);
}

void LinuxCM730::HighPriorityWait() {
    sem_wait_nointr(&m_HighSemID);
}

void LinuxCM730::LowPriorityRelease() {
    sem_post(&m_LowSemID);
}


void LinuxCM730::MidPriorityRelease() {
    sem_post(&m_MidSemID);
}

void LinuxCM730::HighPriorityRelease() {
    sem_post(&m_HighSemID);
}

double LinuxCM730::GetCurrentTime() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);

    return ((double) tv.tv_sec * 1000.0 + (double) tv.tv_usec / 1000.0);
}

void LinuxCM730::SetPacketTimeout(int lenPacket) {
    m_packet_start_time = GetCurrentTime();
    m_packet_wait_time = m_byte_transfer_time * (double) lenPacket + 5.0;
}

bool LinuxCM730::IsPacketTimeout() {
    return GetPacketTime() > m_packet_wait_time;
}

double LinuxCM730::GetPacketTime() {
    double time;

    time = GetCurrentTime() - m_packet_start_time;
    if (time < 0.0)
        m_packet_start_time = GetCurrentTime();

    return time;
}

void LinuxCM730::SetUpdateTimeout(int msec) {
    m_update_start_time = GetCurrentTime();
    m_update_wait_time = msec;
}

bool LinuxCM730::IsUpdateTimeout() {
    return GetUpdateTime() > m_update_wait_time;

}

double LinuxCM730::GetUpdateTime() {
    double time;
    time = GetCurrentTime() - m_update_start_time;
    if (time < 0.0)
        m_update_start_time = GetCurrentTime();

    return time;
}

void LinuxCM730::Sleep(double msec) {
    double start_time = GetCurrentTime();
    double curr_time = start_time;

    do {
        usleep((start_time + msec) - curr_time);
        curr_time = GetCurrentTime();
    } while (curr_time - start_time < msec);
}

bool LinuxCM730::IsDebugEnabled() const {
    return m_debug;
}

void LinuxCM730::EnableDebug(bool debug) {
    m_debug = debug;
}
