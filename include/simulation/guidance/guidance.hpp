#pragma once
#include <cstddef>
#include <Eigen/Dense>
#include "simulation/constants/constants.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/guidance/shared.hpp"

namespace guidance {

    enum class TrajectoryType {
        Stationary,
        Prespecified,
        Interpolated
    };

    inline constexpr std::size_t guidance_state_dim = constants::state_dim + 1;

    using GuidanceStateVector = Eigen::Matrix<double, guidance_state_dim, 1>;

    struct TrajectoryComponents {
        Eigen::MatrixXd v_mat;
        Eigen::MatrixXd w_mat;
        Eigen::MatrixXd eul_mat;
        int n_rows = 0;
    };

    struct Trajectory {
        Eigen::MatrixXd data;
    };

    struct GuidanceProperties {
        TrajectoryType trajectory_type;
        Trajectory trajectory;
        GuidanceSetpoint step(int t, int tf);
    };

    GuidanceStateVector unpack_rigid_body_state(const dynamics::RigidBodyState& xN_t);
    GuidanceSetpoint pack_guidance_setpoint(const GuidanceStateVector& guidance_vec);

}
