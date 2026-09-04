#pragma once
#include <Eigen/Dense>

namespace util
{

	double first_order_lag(double val, double prev_val, double tau, double dt);
	Eigen::Vector3d first_order_lag(const Eigen::Vector3d& val, const Eigen::Vector3d& prev_val, double tau, double dt);
	Eigen::Quaterniond
	first_order_lag(const Eigen::Quaterniond& val, const Eigen::Quaterniond& prev_val, double tau, double dt);

} // namespace util
