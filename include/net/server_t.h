/// \autor arssivka
/// \date 10/31/17

#pragma once


#include <boost/asio.hpp>
#include <set>
#include "server_listener_t.h"

namespace drwn {
    class server_t {
    public:
        server_t() : m_srv_socket(m_io) {
            m_srv_socket.non_blocking(true);
        }

        server_t(const server_t&) = delete;
        server_t& operator=(const server_t&) = delete;

        ~server_t() {
            this->close();
        }

        template <class Option>
        void set_option(const Option& opt) {
            m_srv_socket.set_option(opt);
        }

        template <class Option>
        Option get_option() const {
            Option opt;
            m_srv_socket.get_option(opt);
            return opt;
        }

        void update() {
            using namespace boost::asio;

            streambuf stream;
            read(m_srv_socket, stream, );
        }

        void add_server_listener(server_listener_t& listner) {
            m_listeners.emplace(std::ref(listner));
        }

        void remove_server_listener(server_listener_t& listener) {
            m_listeners.erase(std::ref(listener));
        }

        void listen(boost::asio::ip::udp::endpoint, int port) {
            m_srv_socket = boost::asio::ip::udp::socket();
        }

        boost::asio::ip::udp::endpoint get_endpoint() const {
            return m_srv_socket.local_endpoint();
        }

        void close() {
            m_srv_socket.close();
        }

        bool is_open() const noexcept {
            m_srv_socket.is_open();
        }

        template <class Func>
        void set_complete_function(Func&& func) {

        }

    private:
        boost::asio::io_service m_io;
        boost::asio::ip::udp::socket m_srv_socket;
        std::function<size_t(boost::system::error_code, size_t)> m_;

        std::set<std::reference_wrapper<server_listener_t>> m_listeners;
    };
}



