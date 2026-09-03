#pragma once
#include <Eigen/Dense>
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/dimensions.hpp"

namespace linearization {

    using StateJacobian = constants::MatrixX_T<double, constants::state_dim, constants::state_dim>;
    using InputJacobian = constants::MatrixX_T<double, constants::state_dim, constants::input_dim>;

    using OutputJacobian = constants::MatrixX_T<double, constants::state_dim, constants::state_dim>;
    using DirectFeedthroughJacobian = constants::MatrixX_T<double, constants::state_dim, constants::input_dim>;

    struct LocalLinearization {
        StateJacobian A = StateJacobian::Zero();
        InputJacobian B = InputJacobian::Zero();
        OutputJacobian C = OutputJacobian::Identity();
        DirectFeedthroughJacobian D = DirectFeedthroughJacobian::Zero();
    };

    struct DiscretizedLocalLinearization {
        StateJacobian A = StateJacobian::Zero();
        InputJacobian B = InputJacobian::Zero();
        OutputJacobian C = OutputJacobian::Identity();
        DirectFeedthroughJacobian D = DirectFeedthroughJacobian::Zero();
    };


    using VirtualInputJacobian = constants::MatrixX_T<double, constants::state_dim, constants::virtual_input_dim>;
    using VirtualDirectFeedthroughJacobian = constants::MatrixX_T<double, constants::state_dim, constants::virtual_input_dim>;

    struct VirtualLocalLinearization {
        StateJacobian A_virtual = StateJacobian::Zero();
        VirtualInputJacobian B_virtual = VirtualInputJacobian::Zero();
        OutputJacobian C_virtual = OutputJacobian::Identity();
        VirtualDirectFeedthroughJacobian D_virtual = VirtualDirectFeedthroughJacobian::Zero();
    };
}
