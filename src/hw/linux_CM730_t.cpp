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
#include <log/trivial_logger_t.h>
#include <iomanip>
#include "hw/linux_CM730_t.h"

using namespace drwn;


linux_CM730_t::linux_CM730_t(const char* name) {
    sem_init(&m_low_sem_ID, 0, 1);
    sem_init(&m_mid_sem_ID, 0, 1);
    sem_init(&m_high_sem_ID, 0, 1);
    set_port_name(name);
}

linux_CM730_t::~linux_CM730_t() {
    close_port();
}

void linux_CM730_t::set_port_name(const char* name) {
    strcpy(m_port_name, name);
}

bool linux_CM730_t::open_port() {
    struct termios newtio;
    struct serial_struct serinfo;
    double baudrate = 1000000.0; //bps (1Mbps)

    close_port();

    if (m_debug) LOG_DEBUG << "CM730_t: " << m_port_name << " open ";

    if ((m_socket_fd = open(m_port_name, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
        goto UART_OPEN_ERROR;

    if (m_debug) LOG_DEBUG << "CM730_t: success!";

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
        LOG_DEBUG << std::setprecision(1) << "CM730_t: Set " << baudrate << "bps";

    // Set non-standard baudrate
    if (ioctl(m_socket_fd, TIOCGSERIAL, &serinfo) < 0)
        goto UART_OPEN_ERROR;

    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    serinfo.custom_divisor = serinfo.baud_base / baudrate;

    if (ioctl(m_socket_fd, TIOCSSERIAL, &serinfo) < 0) {
        if (m_debug) LOG_ERROR << "CM730_t: failed!";
        goto UART_OPEN_ERROR;
    }

    if (m_debug) LOG_DEBUG << "CM730_t: success!";

    tcflush(m_socket_fd, TCIFLUSH);

    m_byte_transfer_time = (1000.0 / baudrate) * 12.0;

    return true;

    UART_OPEN_ERROR:
    if (m_debug) LOG_ERROR << "CM730_t: failed!";
    close_port();
    return false;
}

bool linux_CM730_t::set_baud(int baud) {
    struct serial_struct serinfo;
    int baudrate = (int) (2000000.0f / (float) (baud + 1));

    if (m_socket_fd == -1)
        return false;

    if (ioctl(m_socket_fd, TIOCGSERIAL, &serinfo) < 0) {
        LOG_ERROR << "CM730_t: Cannot get serial info";
        return false;
    }

    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    serinfo.custom_divisor = serinfo.baud_base / baudrate;

    if (ioctl(m_socket_fd, TIOCSSERIAL, &serinfo) < 0) {
        LOG_ERROR << "CM730_t: Cannot set serial info";
        return false;
    }

    close_port();
    open_port();

    m_byte_transfer_time = (1000.0f / baudrate) * 12.0f * 8;

    return true;
}

void linux_CM730_t::close_port() {
    if (m_socket_fd != -1)
        close(m_socket_fd);
    m_socket_fd = -1;
}

void linux_CM730_t::clear_port() {
    tcflush(m_socket_fd, TCIFLUSH);
}

int linux_CM730_t::write_port(unsigned char* packet, int numPacket) {
    return write(m_socket_fd, packet, numPacket);
}

int linux_CM730_t::read_port(unsigned char* packet, int numPacket) {
    return read(m_socket_fd, packet, numPacket);
}

void sem_wait_nointr(sem_t* sem) {
    int sem_result, sem_count = 0;
    do {
        sem_result = sem_wait(sem);
    } while ((sem_result == -1) && (errno == EINTR));
}

void linux_CM730_t::low_priority_wait() {
    sem_wait_nointr(&m_low_sem_ID);
}

void linux_CM730_t::mid_priority_wait() {
    sem_wait_nointr(&m_mid_sem_ID);
}

void linux_CM730_t::high_priority_wait() {
    sem_wait_nointr(&m_high_sem_ID);
}

void linux_CM730_t::low_priority_release() {
    sem_post(&m_low_sem_ID);
}


void linux_CM730_t::mid_priority_release() {
    sem_post(&m_mid_sem_ID);
}

void linux_CM730_t::high_priority_release() {
    sem_post(&m_high_sem_ID);
}

double linux_CM730_t::GetCurrentTime() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);

    return ((double) tv.tv_sec * 1000.0 + (double) tv.tv_usec / 1000.0);
}

void linux_CM730_t::set_packet_timeout(int lenPacket) {
    m_packet_start_time = GetCurrentTime();
    m_packet_wait_time = m_byte_transfer_time * (double) lenPacket + 5.0;
}

bool linux_CM730_t::is_packet_timeout() {
    return get_packet_time() > m_packet_wait_time;
}

double linux_CM730_t::get_packet_time() {
    double time;

    time = GetCurrentTime() - m_packet_start_time;
    if (time < 0.0)
        m_packet_start_time = GetCurrentTime();

    return time;
}

void linux_CM730_t::set_update_timeout(int msec) {
    m_update_start_time = GetCurrentTime();
    m_update_wait_time = msec;
}

bool linux_CM730_t::is_update_timeout() {
    return get_update_time() > m_update_wait_time;

}

double linux_CM730_t::get_update_time() {
    double time;
    time = GetCurrentTime() - m_update_start_time;
    if (time < 0.0)
        m_update_start_time = GetCurrentTime();

    return time;
}

void linux_CM730_t::sleep(double msec) {
    double start_time = GetCurrentTime();
    double curr_time = start_time;

    do {
        usleep((start_time + msec) - curr_time);
        curr_time = GetCurrentTime();
    } while (curr_time - start_time < msec);
}

bool linux_CM730_t::IsDebugEnabled() const {
    return m_debug;
}

void linux_CM730_t::EnableDebug(bool debug) {
    m_debug = debug;
}
