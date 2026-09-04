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

		double step(double meas, std::optional<double>& lag_state, double dt);
		Eigen::Vector3d step(const Eigen::Vector3d& meas, std::optional<Eigen::Vector3d>& lag_state, double dt);
		Eigen::Quaterniond
		step(const Eigen::Quaterniond& meas, std::optional<Eigen::Quaterniond>& lag_state, double dt);
	};

} // namespace sensors
