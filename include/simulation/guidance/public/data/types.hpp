#pragma once
#include <cstddef>
#include <Eigen/Dense>
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/dynamics/public/data/types.hpp"

namespace guidance {

    struct GuidanceSetpoint {
        dynamics::TranslationalVelocity vB_BI;
        dynamics::AngularVelocity wB_BI;
        dynamics::EulerAngles eulIB;
    };

    struct AttitudeSetpoint : GuidanceSetpoint {};

    struct VelocitySetpoint : GuidanceSetpoint {};

    struct LinearQuadraticSetpoint : GuidanceSetpoint {};

    struct NonlinearSetpoint : GuidanceSetpoint {};

    enum class TrajectoryType {
        Regulation,
        Tracking,
        Interpolated
    };

    // The guidance setpoint vector stores psi, which is not a state variable, hence the +1
    inline constexpr std::size_t guidance_state_dim = constants::state_dim + 1;

    using GuidanceSetpointVector = constants::MatrixX_T<double, guidance_state_dim, 1>;

    struct TrajectoryComponents {
        Eigen::MatrixXd v_traj;
        Eigen::MatrixXd w_traj;
        Eigen::MatrixXd eul_traj;
        int n_rows = 0;
    };

    struct Trajectory {
        Eigen::MatrixXd data;
    };

    struct GuidanceManagerInput {
        int kf;
    };

    struct GuidanceManagerOutput {
        GuidanceSetpoint setpoint;
    };

}
