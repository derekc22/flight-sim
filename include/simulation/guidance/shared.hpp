#pragma once
#include "simulation/dynamics/shared.hpp"

namespace guidance {

    struct GuidanceSetpoint {
        dynamics::LinearVelocity vB_BI;
        dynamics::AngularVelocity wB_BI;
        dynamics::EulerAngles eulIB;
    };

    struct AxialSetpoint : GuidanceSetpoint {};

    struct VelocitySetpoint : GuidanceSetpoint {};

    struct LinearFullStateFeedbackSetpoint : GuidanceSetpoint {};

    struct NonlinearSetpoint : GuidanceSetpoint {};

}
