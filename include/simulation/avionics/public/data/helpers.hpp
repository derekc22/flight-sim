#pragma once
#include "simulation/avionics/public/data/types.hpp"

namespace avionics {

    AvionicsGroundTruth build_avionics_gt(
        const dynamics::RigidBodyState& Xt,
        const dynamics::RigidBodyState& XEt,
        const aerodynamics::AerodynamicState& aero_t,
        const atmospheric::StaticAtmosphericState& atm_t,
        const geography::GeographicState& geo_t
    );

    dynamics::RigidBodyState get_state_from_avionics(
        const sensors::SensorMeasurements& sensor_meas, 
        const AvionicsMeasurements& avionics_meas, 
        const Settings& avionics_settings
    );

}
