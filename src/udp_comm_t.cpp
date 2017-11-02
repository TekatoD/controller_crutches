/**
 * @file UdpComm.cpp
 * Implements a wrapper for a UDP socket.
 * @author Armin Burchardt
 */

#include "udp_comm_t.h"

#include <iostream>
#include <cassert>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <ifaddrs.h>
#include <errno.h>


udp_comm_t::udp_comm_t()
        : m_sock(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)),
          m_target((struct sockaddr*) (new struct sockaddr_in)) {
    assert(m_sock != -1);
}


udp_comm_t::~udp_comm_t() {
    close(m_sock);
}


bool udp_comm_t::resolve(const char* addrStr, int port, struct sockaddr_in* addr) {
    memset(addr, 0, sizeof(struct sockaddr_in));
    addr->sin_family = AF_INET;
    addr->sin_port = htons(static_cast<unsigned short>(port));
    if (inet_pton(AF_INET, addrStr, &(addr->sin_addr.s_addr)) != 1) {
        std::cerr << addrStr << " is not a valid dotted ipv4 address" << std::endl;
        return false;
    }

    return true;
}


bool udp_comm_t::set_target(const char* addrStr, int port) {
    struct sockaddr_in* addr = (struct sockaddr_in*) m_target;
    return resolve(addrStr, port, addr);
}


bool udp_comm_t::set_blocking(bool block) {
    if (block)
        return fcntl(m_sock, F_SETFL, 0) != -1;
    else
        return fcntl(m_sock, F_SETFL, O_NONBLOCK) != -1;
}


bool udp_comm_t::set_loopback(bool yes_no) {
    char val = yes_no ? 1 : 0;
    if (setsockopt(m_sock, IPPROTO_IP, IP_MULTICAST_LOOP, &val, sizeof(char)) < 0) {
        std::cerr << "could not set ip_multicast_loop to " << val << std::endl;
        return false;
    }
    return true;
}


bool udp_comm_t::set_broadcast(bool enable) {
    int yes = enable ? 1 : 0;
    if (setsockopt(m_sock, SOL_SOCKET, SO_BROADCAST,
                   (const char*) &yes, sizeof(yes)) == 0)
        return true;
    else {
        std::cerr << "udp_comm_t::set_broadcast() failed: " << strerror(errno) << std::endl;
        return false;
    }
}


bool udp_comm_t::bind(const char* addr_str, int port) {
    static const int yes = 1;
    struct sockaddr_in addr;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<unsigned short>(port));
    addr.sin_family = AF_INET;

    if (inet_pton(AF_INET, addr_str, &(addr.sin_addr)) <= 0) {
        std::cerr << "udp_comm_t::bind() failed: invalid address " << addr_str << std::endl;
        return false;
    }

#ifdef SO_REUSEADDR
    if (-1 == setsockopt(m_sock, SOL_SOCKET, SO_REUSEADDR, (const char*) &yes, sizeof(yes)))
        std::cerr << "udp_comm_t: could not set SO_REUSEADDR" << std::endl;
#endif
#ifdef SO_REUSEPORT
    if (-1 == setsockopt(m_sock, SOL_SOCKET, SO_REUSEPORT, (const char*) &yes, sizeof(yes)))
        std::cerr << "udp_comm_t: could not set SO_REUSEPORT" << std::endl;
#endif
    if (-1 == ::bind(m_sock, (struct sockaddr*) &addr, sizeof(struct sockaddr_in))) {
        std::cerr << "udp_comm_t: bind failed: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}


int udp_comm_t::read(char* data, int len) {
    return ::recv(m_sock, data, len, 0);
}


int udp_comm_t::read(char* data, int len, sockaddr_in& from) {
    socklen_t fromLen = sizeof(from);
    return ::recvfrom(m_sock, data, len, 0, (sockaddr*) &from, &fromLen);
}


bool udp_comm_t::write(const char* data, const int len) {
    return ::sendto(m_sock, data, len, 0,
                    m_target, sizeof(struct sockaddr_in)) == len;
}


const char* udp_comm_t::get_wifi_broadcast_address() {
    struct ifaddrs* ifAddrStruct = NULL;
    struct ifaddrs* ifa = NULL;

    //determine ip address
    getifaddrs(&ifAddrStruct);
    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        // manpage getifaddrs    // check it is IP4
        if (ifa->ifa_addr != NULL && ifa->ifa_addr->sa_family == AF_INET) {
            std::string interfaceName(ifa->ifa_name);
            if (interfaceName.find("wlan") != std::string::npos) {
                in_addr_t mask = ((struct sockaddr_in*) ifa->ifa_netmask)->sin_addr.s_addr;
                in_addr_t addr = ((struct sockaddr_in*) ifa->ifa_addr)->sin_addr.s_addr;
                in_addr_t bcastAddr = ~mask | addr;

                struct in_addr bcast_addr;
                bcast_addr.s_addr = bcastAddr;
                static char buffer[INET_ADDRSTRLEN];
                inet_ntop(AF_INET,
                          &bcast_addr,
                          buffer,
                          INET_ADDRSTRLEN);
                return buffer;
            }
        }
    }
    return "255.255.255.255";
}
