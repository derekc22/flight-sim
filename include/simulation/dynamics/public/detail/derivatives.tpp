#pragma once
#include "simulation/constants/public/linalg.hpp"
#include "simulation/util/public/cppad.hpp"
#include "simulation/util/public/trig.hpp"

namespace dynamics
{

	// Translational dynamics (non-rotating earth), velocity expressed in body coordinates
	// vB_dot = (1/mass) * FB_net - wB_BI x vB
	template <typename T>
	constants::Vector3_T<T> ddtB_vB_BI_T(
		const constants::Vector3_T<T>& vB,
		const constants::Vector3_T<T>& wB_BI,
		double mass,
		const constants::Vector3_T<T>& FB_net)
	{
		return (FB_net / T(mass)) - wB_BI.cross(vB);
	}

	// Rotational dynamics, angular velocity expressed in body coordinates
	// J * w_dot + w x (J*w) = M
	// w_dot = J^{-1} * (M - w x (J*w))
	template <typename T>
	constants::Vector3_T<T> ddtB_wB_BI_T(
		const constants::Vector3_T<T>& wB_BI,
		const Eigen::Matrix3d& J,
		const constants::Vector3_T<T>& MB_net)
	{
		const constants::Matrix3_T<T> J_T = J.cast<T>();
		const constants::Matrix3_T<T> J_inv_T = J.inverse().cast<T>();
		const constants::Vector3_T<T> rhs = MB_net - wB_BI.cross(J_T * wB_BI);
		return J_inv_T * rhs;
	}

	template <typename T>
	constants::Matrix3_T<T> wB_BI_to_eul_dot_mat_T(
		const T& theta,
		const T& phi)
	{
		constants::Matrix3_T<T> T_mat;
		T_mat << T(1.0), util::sin(phi) * util::tan(theta), util::cos(phi) * util::tan(theta), T(0.0), util::cos(phi),
			-util::sin(phi), T(0.0), util::sin(phi) * util::sec(theta), util::cos(phi) * util::sec(theta);
		return T_mat;
	}

	template <typename T>
	constants::Vector3_T<T> wB_BI_to_eul_dot_T(
		const constants::Vector3_T<T>& wB_BI,
		const T& theta,
		const T& phi)
	{
		return wB_BI_to_eul_dot_mat_T<T>(theta, phi) * wB_BI;
	}

} // namespace dynamics
