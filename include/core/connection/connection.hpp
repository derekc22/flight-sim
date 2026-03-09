#pragma once
#include <netinet/in.h>
#include <optional>
#include <string>
#include "core/messages/messages.hpp"

namespace connection {

    struct UDPIn {
        UDPIn(const std::string& host, int port);
        ~UDPIn();

        UDPIn(const UDPIn&) = delete;
        UDPIn& operator=(const UDPIn&) = delete;

        bool send(const messages::FlightGearMessageIn& msg);

        int fd_{-1};
        sockaddr_in dst_{};
    };

    struct UDPOut {
        explicit UDPOut(int port);
        ~UDPOut();

        UDPOut(const UDPOut&) = delete;
        UDPOut& operator=(const UDPOut&) = delete;

        std::optional<messages::FlightGearMessageOut> try_receive();

        int fd_{-1};
    };


}
