#pragma once
#include "simulation/dynamics/shared.hpp"

namespace guidance {

    struct GuidanceSetpoint {
        const dynamics::LinearVelocity vB_BI;
        const dynamics::AngularVelocity wB_BI;
        const dynamics::EulerAngles eulIB;
    };

    struct AxialSetpoint : GuidanceSetpoint {};

    struct VelocitySetpoint : GuidanceSetpoint {};

    struct LinearFullStateFeedbackSetpoint : GuidanceSetpoint {};

    struct NonlinearSetpoint : GuidanceSetpoint {};

}
