#pragma once
#include <Eigen/Dense>
#include "simulation/geography/shared.hpp"
#include "simulation/dynamics/shared.hpp"

namespace messages {

    struct FlightGearMessageIn {
        double altitude;    // [ft]
        double latitude;    // [deg]
        double longitude;   // [deg]
        double roll;        // [deg]
        double pitch;       // [deg]
        double heading;     // [deg]
    };

    struct FlightGearMessageOut {
        float wind_heading;  // [deg] 
        float wind_speed;    // [kt]
    };

    FlightGearMessageIn process_in_pkt(const geography::GeographicState& geographic_state, const dynamics::EulerAngles& attitude);


}