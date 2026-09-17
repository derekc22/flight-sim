#pragma once
#include <Eigen/Dense>
#include <optional>
#include <random>

namespace sensors
{

	/** @note For simplicity, the internal dynamics of some sensors will remain unmodelled 
        Instead, these sensors will simply output a noised, lagged version of the corresponding ground-truth quantity
    */
	struct Sensor {
		double mean;
		double stddev;
		double bias;
		Eigen::Vector3d bias_3d;
		std::mt19937 gen{std::random_device{}()};
		std::normal_distribution<double> dist;
		double tau;

		Sensor(double mean, double stddev, double bias, const Eigen::Vector3d& bias3d, double tau);

		/**
		 * @brief Applies first-order lag, scalar bias, and Gaussian noise to a measurement.
		 *
		 * An empty lag state is initialized from @p meas before filtering.
		 *
		 * @param[in] meas Ground-truth scalar measurement in its native units.
		 * @param[in,out] lag_state Previous lagged value, updated with the current lagged value.
		 * @param[in] dt Measurement interval [s].
		 * @return Lagged, biased, and noised scalar measurement in its native units.
		 */
		double step(double meas, std::optional<double>& lag_state, double dt);

		/**
		 * @brief Applies first-order lag, vector bias, and independent Gaussian noise to a vector measurement.
		 *
		 * An empty lag state is initialized from @p meas before filtering.
		 *
		 * @param[in] meas Ground-truth vector measurement in its native units.
		 * @param[in,out] lag_state Previous lagged vector, updated with the current lagged vector.
		 * @param[in] dt Measurement interval [s].
		 * @return Lagged, biased, and noised vector measurement in its native units.
		 */
		Eigen::Vector3d step(const Eigen::Vector3d& meas, std::optional<Eigen::Vector3d>& lag_state, double dt);

		/**
		 * @brief Applies first-order lag, rotational bias, and rotational Gaussian noise to a quaternion.
		 *
		 * Lag is applied with shortest-path spherical interpolation. Bias and noise
		 * rotations are post-multiplied onto the lagged quaternion, and the result is
		 * normalized. An empty lag state is initialized from @p meas.
		 *
		 * @param[in] meas Ground-truth orientation quaternion [-].
		 * @param[in,out] lag_state Previous lagged quaternion, updated before bias and noise are applied.
		 * @param[in] dt Measurement interval [s].
		 * @return Normalized lagged, biased, and noised orientation quaternion [-].
		 */
		Eigen::Quaterniond
		step(const Eigen::Quaterniond& meas, std::optional<Eigen::Quaterniond>& lag_state, double dt);
	};

} // namespace sensors
