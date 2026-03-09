#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include "core/connection/connection.hpp"

namespace connection {

    UDPIn::UDPIn(const std::string& host, int port) {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) return;

        dst_.sin_family = AF_INET;
        dst_.sin_port = htons(static_cast<uint16_t>(port));
        ::inet_pton(AF_INET, host.c_str(), &dst_.sin_addr);
    }

    UDPIn::~UDPIn() {
        if (fd_ >= 0) ::close(fd_);
    }

    bool UDPIn::send(const messages::FlightGearMessageIn& msg) {
        if (fd_ < 0) return false;

        char buffer[256];
        const int n = std::snprintf(
            buffer, sizeof(buffer),
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            msg.altitude,
            msg.latitude,
            msg.longitude,
            msg.roll,
            msg.pitch,
            msg.heading
        );

        if (n <= 0) return false;

        const ssize_t sent = ::sendto(
            fd_, buffer, static_cast<size_t>(n), 0,
            reinterpret_cast<sockaddr*>(&dst_), sizeof(dst_)
        );

        return sent == n;
    }

    UDPOut::UDPOut(int port) {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) return;

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port));
        addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            ::close(fd_);
            fd_ = -1;
            return;
        }

        const int flags = ::fcntl(fd_, F_GETFL, 0);
        if (flags >= 0) {
            ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
        }
    }

    UDPOut::~UDPOut() {
        if (fd_ >= 0) ::close(fd_);
    }

    std::optional<messages::FlightGearMessageOut> UDPOut::try_receive() {
        if (fd_ < 0) return std::nullopt;

        char buffer[256];
        sockaddr_in src{};
        socklen_t srclen = sizeof(src);

        const ssize_t n = ::recvfrom(
            fd_, buffer, sizeof(buffer) - 1, 0,
            reinterpret_cast<sockaddr*>(&src), &srclen
        );

        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return std::nullopt;
            }
            return std::nullopt;
        }

        buffer[n] = '\0';

        messages::FlightGearMessageOut msg{};
        if (std::sscanf(buffer, "%f,%f", &msg.wind_heading, &msg.wind_speed) != 2) {
            return std::nullopt;
        }

        return msg;
    }
}
