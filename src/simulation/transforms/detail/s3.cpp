#include <Eigen/Dense>
#include <stdexcept>
#include <algorithm>
#include <string>
#include <vector>
#include "simulation/transforms/private/detail/s3.hpp"
#include "simulation/transforms/public/detail/s3.hpp"
#include "simulation/util/public/math.hpp"

namespace transforms {

	// Recall that the premise of an "intrinsic" vector rotation (as opposed to an intrinsic frame rotations) is rejected
	// Thus, the "intr" branch of `eul_to_quatR` and `quatquatR_to_eul` do not exist
	// That is, this function DOES NOT have a 'type' argument and appropriately ONLY calls `eul_to_quatR_extr` internally
	// Once again, the 'types' argument is omitted in eul_to_quatR and quatR_to_eul because including it would imply that active rotation matrices, R, can apply intrinsic rotations
	// This is not true. Active rotation matrices CANNOT apply intrinsic rotations - they can ONLY apply extrinsic rotations, hence why `eul_to_quatR` automtically calls eul_to_quatR_extr and does not support the option to build an `intrinsic` R (which, again, DOES NOT EXIST)
	Eigen::Quaterniond eul_to_quatR(
	    double a,
	    double b,
	    double c,
	    EulerOrder order)
	{
		return eul_to_quatR_extr(a, b, c, order);
	}

	Eigen::Quaterniond eul_to_quatC(
	    double a,
	    double b,
	    double c,
	    EulerOrder order,
	    RotationType type)
	{
		if (type == RotationType::Extrinsic)
			return eul_to_quatC_extr(a, b, c, order);
		if (type == RotationType::Intrinsic)
			return eul_to_quatC_intr(a, b, c, order);
		throw std::invalid_argument("Unsupported type");
	}

	Eigen::Vector3d quatR_to_eul(
	    const Eigen::Quaterniond& qR,
	    EulerOrder order)
	{
		return quatR_to_eul_extr(qR, order);
	}

	Eigen::Vector3d quatC_to_eul(
	    const Eigen::Quaterniond& qC,
	    EulerOrder order,
	    RotationType type)
	{
		if (type == RotationType::Extrinsic)
			return quatC_to_eul_extr(qC, order);
		if (type == RotationType::Intrinsic)
			return quatC_to_eul_intr(qC, order);
		throw std::invalid_argument("Unsupported type");
	}

	Eigen::Matrix3d quat_to_rot(
	    const Eigen::Quaterniond& q)
	{
		return Eigen::Matrix3d(normalize_and_canonicalize(q));
	}

	Eigen::Quaterniond normalize_and_canonicalize(
	    Eigen::Quaterniond q)
	{
		q.normalize();
		// Canonicalize sign: q and -q represent the same rotation
		if (q.w() < 0.0)
			q.coeffs() *= -1.0; // coeffs() is (x,y,z,w)
		return q;
	}
} // namespace transforms
