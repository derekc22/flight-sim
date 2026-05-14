#include <Eigen/Dense>
#include "core/messages/messages.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/util/public.hpp"

namespace messages {

    FlightGearMessageIn process_in_pkt(const geography::GeographicState& geographic_state, const dynamics::EulerAngles& attitude) {
        return FlightGearMessageIn {
            .altitude = util::m_to_ft(geographic_state.alt.data),
            .latitude = util::rad_to_deg(geographic_state.lat.data),
            .longitude = util::rad_to_deg(geographic_state.lon.data),
            .roll = util::rad_to_deg(attitude.phi()),
            .pitch = util::rad_to_deg(attitude.theta()),
            .heading = util::rad_to_deg(attitude.psi())
        };
    }



}
