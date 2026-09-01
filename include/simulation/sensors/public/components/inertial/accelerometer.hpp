#pragma once
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace sensors {

    struct Accelerometer : Sensor {
        std::optional<Eigen::Vector3d> prev_fB_lag;
        TranslationalAccelerationMeasurement measure(const dynamics::TranslationalAcceleration& fB, double dt);
        // Note: fB = aB_BI - gB 
        //          = FB_net/m - gB
        // Where 
        // fB is specific force 
        // aB_BI is inertial acceleration (the inerital derivative of velocity - as opposed to the body derivative of velocity, vB_BI_dot)
        // That is, an accelerometer measures specific force - which is inertial acceleration minus the contribution from gravity and expressed in the body frame
    };

}
