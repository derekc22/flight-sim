#include "core/messages/public/data/helpers.hpp"
#include "simulation/atmospheric/public/detail/wind.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/geography/public/data/types.hpp"
#include "simulation/util/public/trig.hpp"
#include "simulation/util/public/units.hpp"

namespace messages {

    FlightGearMessageIn process_in_msg(const geography::GeographicState& geo, const dynamics::EulerAngles& attitude) {
        return FlightGearMessageIn {
            .altitude_ft = util::m_to_ft(geo.alt.data),
            .latitude_deg = util::rad_to_deg(geo.lat.data),
            .longitude_deg = util::rad_to_deg(geo.lon.data),
            .roll_deg = util::rad_to_deg(attitude.phi()),
            .pitch_deg = util::rad_to_deg(attitude.theta()),
            .heading_deg = util::rad_to_deg(attitude.psi())
        };
    }

    ProcessedFlightGearMessageOut process_out_msg(const FlightGearMessageOut& out_msg) {
        atmospheric::Wind wind = atmospheric::build_wind(
            out_msg.wind_heading_deg, 
            out_msg.wind_speed_kt
        );
        geography::GroundElevation ground_elevation{ util::ft_to_m(out_msg.ground_elev_ft) };

        return { .wind = wind, .ground_elevation = ground_elevation };
    }
}
