#pragma once
#include "simulation/dynamics/dynamics.hpp"

namespace guidance {

    struct GuidanceSetpoint {
        dynamics::LinearVelocity vB_BI;
        dynamics::AngularVelocity wB_BI;
        dynamics::EulerAngles eulIB;
    };

    struct AxialSetpoint : GuidanceSetpoint {
        // uses w, eul;
    };

    struct VelocitySetpoint : GuidanceSetpoint {
        // uses v
    };

    struct LinearFullStateFeedbackSetpoint : GuidanceSetpoint {
        // uses v, w, eul;
    };

    struct NonlinearSetpoint : GuidanceSetpoint {
        // uses v, w, eul;
    };

    enum class TrajectoryType {
        Stationary,
        Prespecified,
        Interpolated
    };

    inline constexpr std::size_t guidance_state_dim = constants::state_dim + 1; 
    using GuidanceStateVector = Eigen::Matrix<double, guidance_state_dim, 1>;

    GuidanceStateVector unpack_rigid_body_state(const dynamics::RigidBodyState& xN_t);
    GuidanceSetpoint pack_guidance_setpoint(const GuidanceStateVector& guidance_vec);

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

}