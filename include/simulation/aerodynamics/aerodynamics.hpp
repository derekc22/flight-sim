#pragma once
#include <cstddef>
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include "simulation/aerodynamics/public.hpp"

#include "simulation/actuators/surface/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/structural/public.hpp"
#include "simulation/util/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"

namespace aerodynamics {

    struct LiftCoefficient {
        double data;
    };

    struct DragCoefficient {
        double data;
    };

    struct MomentCoefficient {
        double data;
    };

    struct SurfaceKinematics {
        Eigen::Vector3d rB_ac = constants::Zero3;
        Eigen::Vector3d vB_rel = constants::Zero3;
        double V = 0.0;
        double qbar = 0.0;
        double alpha = 0.0;
        double p_hat = 0.0;
        double q_hat = 0.0;
        double r_hat = 0.0;
    };

    struct SurfaceCoefficients {
        LiftCoefficient CL;
        DragCoefficient CD;
        MomentCoefficient CM;
    };
}
