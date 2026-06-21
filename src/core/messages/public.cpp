#include "core/messages/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/util/public.hpp"

namespace messages {

    FlightGearMessageIn process_in_pkt(const geography::GeographicState& geo, const dynamics::EulerAngles& attitude) {
        return FlightGearMessageIn {
            .altitude_ft = util::m_to_ft(geo.alt.data),
            .latitude_deg = util::rad_to_deg(geo.lat.data),
            .longitude_deg = util::rad_to_deg(geo.lon.data),
            .roll_deg = util::rad_to_deg(attitude.phi()),
            .pitch_deg = util::rad_to_deg(attitude.theta()),
            .heading_deg = util::rad_to_deg(attitude.psi())
        };
    }

    ProcessedFlightGearMessageOut process_out_pkt(const FlightGearMessageOut& out_pkt) {
        atmospheric::Wind wind = atmospheric::build_wind(
            out_pkt.wind_heading_deg, 
            out_pkt.wind_speed_kt
        );
        geography::GroundElevation ground_elevation{ util::ft_to_m(out_pkt.ground_elev_ft) };

        return { .wind = wind, .ground_elevation = ground_elevation };
    }
}
