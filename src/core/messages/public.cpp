#include "core/messages/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/util/public.hpp"

namespace messages {

    FlightGearMessageIn process_in_pkt(const geography::GeographicState& geographic_state, const dynamics::EulerAngles& attitude) {
        return FlightGearMessageIn {
            .altitude_ft = util::m_to_ft(geographic_state.alt.data),
            .latitude_deg = util::rad_to_deg(geographic_state.lat.data),
            .longitude_deg = util::rad_to_deg(geographic_state.lon.data),
            .roll_deg = util::rad_to_deg(attitude.phi()),
            .pitch_deg = util::rad_to_deg(attitude.theta()),
            .heading_deg = util::rad_to_deg(attitude.psi())
        };
    }

    ProcessedFlightGearMessageOut process_out_pkt(const std::optional<FlightGearMessageOut>& out_pkt) {
        if (!out_pkt.has_value()) { return {}; }
        atmospheric::Wind wind = atmospheric::build_wind(
            out_pkt->wind_heading_deg, 
            out_pkt->wind_speed_kt
        );
        double ground_elev = util::ft_to_m(out_pkt->ground_elev_ft);

        return { .wind = wind, .ground_elev = ground_elev };
    }
}
