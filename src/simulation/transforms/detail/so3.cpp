#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include "simulation/util/public/math.hpp"
#include "simulation/transforms/private/detail/so3.hpp"
#include "simulation/transforms/public/detail/s3.hpp"
#include "simulation/transforms/public/detail/so3.hpp"

namespace transforms {

	Eigen::Matrix3d C_from_R(
	    const Eigen::Matrix3d& R)
	{
		return R.transpose();
	}

	Eigen::Matrix3d R_from_C(
	    const Eigen::Matrix3d& C)
	{
		return C.transpose();
	}

	// Recall that the premise of an "intrinsic" vector rotation (as opposed to an intrinsic frame rotations) is rejected
	// Thus, the "intr" branch of `eul_to_R` and `R_to_eul` do not exist
	// That is, this function DOES NOT have a 'type' argument and appropriately ONLY calls `eul_to_R_extr` internally
	// Once again, the 'types' argument is omitted in eul_to_R and R_to_eul because including it would imply that active rotation matrices, R, can apply intrinsic rotations
	// This is not true. Active rotation matrices CANNOT apply intrinsic rotations - they can ONLY apply extrinsic rotations, hence why `eul_to_R` automatically calls eul_to_R_extr and does not support the option to build an `intrinsic` R (which, again, DOES NOT EXIST)
	Eigen::Matrix3d eul_to_R(
	    double a,
	    double b,
	    double c,
	    EulerOrder order)
	{
		return eul_to_R_extr(a, b, c, order);
	}

	Eigen::Matrix3d eul_to_C(
	    double a,
	    double b,
	    double c,
	    EulerOrder order,
	    RotationType type)
	{
		if (type == RotationType::Extrinsic)
			return eul_to_C_extr(a, b, c, order);
		if (type == RotationType::Intrinsic)
			return eul_to_C_intr(a, b, c, order);
		throw std::invalid_argument("Unsupported type");
	}

	Eigen::Vector3d R_to_eul(
	    const Eigen::Matrix3d& R,
	    EulerOrder order)
	{
		return R_to_eul_extr(R, order);
	}

	Eigen::Vector3d C_to_eul(
	    const Eigen::Matrix3d& C,
	    EulerOrder order,
	    RotationType type)
	{
		if (type == RotationType::Extrinsic)
			return C_to_eul_extr(C, order);
		if (type == RotationType::Intrinsic)
			return C_to_eul_intr(C, order);
		throw std::invalid_argument("Unsupported type");
	}

	Eigen::Quaterniond rot_to_quat(
	    const Eigen::Matrix3d& rot)
	{
		return normalize_and_canonicalize(Eigen::Quaterniond(rot));
	}

} // namespace transforms
