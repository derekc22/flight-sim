#include <Eigen/Dense>
#include "core/messages/messages.hpp"
#include "simulation/global/global.hpp"
#include "simulation/geography/geography.hpp"
#include "simulation/dynamics/dynamics.hpp"

namespace messages {

    FlightGearMessageIn process_in_pkt(const geography::GeographicState& geographic_state, const dynamics::EulerAngles& attitude) {
        return FlightGearMessageIn {
            .altitude = global::m_to_ft(geographic_state.alt.data),
            .latitude = global::rad_to_deg(geographic_state.lat.data),
            .longitude = global::rad_to_deg(geographic_state.lon.data),
            .roll = global::rad_to_deg(attitude.phi()),
            .pitch = global::rad_to_deg(attitude.theta()),
            .heading = global::rad_to_deg(attitude.psi())
        };
    }



}
