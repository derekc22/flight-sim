#pragma once
#include "simulation/geography/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/constants/public.hpp"

namespace messages {

    struct FlightGearMessageIn {
        double altitude_ft;     // [ft]
        double latitude_deg;    // [deg]
        double longitude_deg;   // [deg]
        double roll_deg;        // [deg]
        double pitch_deg;       // [deg]
        double heading_deg;     // [deg]
    };

    struct FlightGearMessageOut {
        float wind_heading_deg;  // [deg] 
        float wind_speed_kt;     // [kt]
        float ground_elev_ft;    // [ft]
    };

    struct ProcessedFlightGearMessageOut {
        atmospheric::Wind wind{ constants::Zero3 };
        geography::GroundElevation ground_elevation{ 0.0 };
    };

    FlightGearMessageIn process_in_msg(const geography::GeographicState& geo, const dynamics::EulerAngles& attitude);
    ProcessedFlightGearMessageOut process_out_msg(const FlightGearMessageOut& out_msg);

}