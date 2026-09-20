#pragma once
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/control/public/data/types.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/linearization/public/data/types.hpp"
#include "simulation/operating/public/data/types.hpp"
#include "simulation/trim/public/data/types.hpp"

#include <Eigen/Dense>

namespace estimation
{
	using StateEstimateErrorCovariance = constants::MatrixX_T<double, constants::nx, constants::nx>;
	using MeasurementNoiseCovariance = constants::MatrixX_T<double, constants::nx, constants::nx>;
	using ProcessNoiseCovariance = constants::MatrixX_T<double, constants::nx, constants::nx>;
	using KalmanGain = constants::MatrixX_T<double, constants::nx, constants::nx>;

	enum class EstimatorType {
		None,
		LinearKalmanFilter,
		ExtendedKalmanFilter
	};

	struct LinearKalmanFilterInput {
		dynamics::RigidBodyState Yt;
		operating::OperatingPoint_T<double> operating_point;
		linearization::LocalLinearization lin_sol;
		actuators::ActuatorInputs_T<double> u_actual_t_1;
	};

	struct ExtendedKalmanFilterInput {
		dynamics::RigidBodyState Yt;
		actuators::ActuatorInputs_T<double> u_actual_t_1;
		const autodiff::AutoDiffModel& model;
		operating::OperatingConditions conditions;
	};

	struct KalmanState {
		dynamics::StateVector_T<double> zt; // state estimate
		StateEstimateErrorCovariance Pt;	// state estimate error covariance matrix
	};

	struct KalmanFilterParameters {
		StateEstimateErrorCovariance P0; // initial state estimate error covariance matrix
		MeasurementNoiseCovariance Q;	 // measurement noise covariance matrix
		ProcessNoiseCovariance R;		 // process noise covariance matrix
	};

	struct LinearKalmanFilterParameters : KalmanFilterParameters {};

	struct ExtendedKalmanFilterParameters : KalmanFilterParameters {};

	struct EstimationManagerInput {
		const dynamics::RigidBodyState& Yt;
		const trim::TrimSolution& trim_sol;
		const linearization::LocalLinearization& lin_sol;
		const autodiff::AutoDiffModel& model;
		const control::ControlOutput& u_actual_t_1;
		const operating::OperatingConditions& conditions;
		double dt;
	};

	struct EstimationManagerOutput {
		dynamics::RigidBodyState Zt; // state estimate
	};

} // namespace estimation
