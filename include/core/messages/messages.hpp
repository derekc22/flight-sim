#pragma once
#include <Eigen/Dense>
#include "simulation/dynamics/dynamics.hpp"

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



}