#pragma once
#include "simulation/avionics/public/components/adc.hpp"
#include "simulation/avionics/public/components/ahrs.hpp"
#include "simulation/avionics/public/components/ins.hpp"

namespace avionics {

    struct Avionics {
        AirDataComputer ADC;
        AttitudeHeadingReferenceSystem AHRS;
        InertialNavigationSystem INS;
    };

}
