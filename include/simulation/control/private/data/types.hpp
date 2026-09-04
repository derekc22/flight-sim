#pragma once
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/linalg.hpp"

#include <Eigen/Dense>
#include <optional>

namespace control
{

	struct PIDPolicyInput {
		double x;
		double x_des;
		std::optional<double> x_dot;
		double delta_mu;
	};

	struct PIDPolicyParameters {
		double Kp;
		double Ki;
		double Kd;
		double tau;
	};

	struct LinearQuadraticPolicyInput {
		Eigen::VectorXd zt;
		Eigen::MatrixXd A_virtual;
		constants::MatrixX_T<double, Eigen::Dynamic, constants::virtual_input_dim> B_virtual;
	};

	struct LinearQuadraticPolicyParameters {
		Eigen::MatrixXd Q;
		Eigen::MatrixXd R;
		std::optional<Eigen::MatrixXd> K;
	};

	struct CareSolution {
		Eigen::MatrixXd P;
		Eigen::MatrixXd residual;
		double rcond = 0.0;
	};

} // namespace control
