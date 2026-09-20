#include "simulation/estimation/public/components/lkf.hpp"

#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/dynamics/public/data/helpers.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/estimation/public/data/helpers.hpp"
#include "simulation/linearization/public/detail/discretization.hpp"

#include <Eigen/Dense>

namespace estimation
{

	LinearKalmanFilter::LinearKalmanFilter(
		const LinearKalmanFilterParameters& params)
		: params(params)
	{
	}

	dynamics::RigidBodyState LinearKalmanFilter::step(
		const LinearKalmanFilterInput& input,
		double dt)
	{
		dynamics::StateVector_T<double> yt_deviation =
			dynamics::unpack_state(input.Yt) - dynamics::unpack_state_T(input.operating_point.state);

		actuators::ActuatorInputsVector_T<double> ut_1_deviation =
			actuators::unpack_actuator_inputs_T(input.u_actual_t_1) -
			actuators::unpack_actuator_inputs_T(input.operating_point.input);

		linearization::DiscretizedLocalLinearization lin_sol_k = linearization::discretize(input.lin_sol, dt);

		if (!state.has_value()) {
			state = KalmanState{.zt = yt_deviation, .Pt = params.P0};
		} else {
			state = predict(lin_sol_k, ut_1_deviation);
			state = correct(yt_deviation, lin_sol_k.C);
		}

		dynamics::StateVector_T<double> x_trim = dynamics::unpack_state_T(input.operating_point.state);
		dynamics::StateVector_T<double> zt_full =
			state.value().zt + x_trim; // LKF predicts deviation state, so trim state is added back

		dynamics::RigidBodyState Zt = make_kalman_state_estimate(input.Yt, zt_full);

		return Zt;
	}

	KalmanState LinearKalmanFilter::predict(
		const linearization::DiscretizedLocalLinearization& lin_sol_k,
		const actuators::ActuatorInputsVector_T<double>& ut_1)
	{
		KalmanState prev = state.value();

		dynamics::StateVector_T<double> zt_bar = lin_sol_k.A * prev.zt + lin_sol_k.B * ut_1;

		constants::MatrixX_T<double, constants::nx, constants::nx> Pt_bar =
			lin_sol_k.A * prev.Pt * lin_sol_k.A.transpose() + params.R;

		return {.zt = zt_bar, .Pt = Pt_bar};
	}

	KalmanState LinearKalmanFilter::correct(
		const dynamics::StateVector_T<double>& yt,
		const linearization::OutputJacobian& C)
	{
		KalmanState pred = state.value();

		// Kalman gain
		constants::MatrixX_T<double, constants::nx, constants::nx> Kt =
			pred.Pt * C.transpose() * (C * pred.Pt * C.transpose() + params.Q).inverse();

		// Innovation
		dynamics::StateVector_T<double> Lt = yt - C * pred.zt;

		dynamics::StateVector_T<double> zt = pred.zt + Kt * Lt;

		constants::MatrixX_T<double, constants::nx, constants::nx> I = constants::I_T<double, constants::nx>;

		constants::MatrixX_T<double, constants::nx, constants::nx> Pt =
			(I - Kt * C) * pred.Pt * (I - Kt * C).transpose() + Kt * params.Q * Kt.transpose();

		return {.zt = zt, .Pt = Pt};
	}

} // namespace estimation
