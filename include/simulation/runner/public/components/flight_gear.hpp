#pragma once
#include "core/connection/public/udp.hpp"
#include "core/messages/public/data/helpers.hpp"
#include "simulation/runner/public/data/types.hpp"

namespace runner {

    struct FlightGear {
        // initialize udp out cache
        messages::ProcessedFlightGearMessageOut cached_msg_out{};

        connection::UDPOut udp_out;
        connection::UDPIn udp_in;

        FlightGear();
        FlightGearOutput receive(const FlightGearInput& input);
        void send(const FlightGearSendInput& input);
    };

}
