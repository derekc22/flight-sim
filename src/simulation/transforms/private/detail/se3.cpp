#include <Eigen/Dense>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include "simulation/constants/public/linalg.hpp"
#include "simulation/transforms/private/detail/se3.hpp"
#include "simulation/transforms/public/detail/se3.hpp"

namespace transforms {

	Eigen::Matrix4d make_HR_translate_first(
	    const Eigen::Matrix3d& R,
	    const Eigen::Vector3d& d)
	{
		Eigen::Matrix4d H = Eigen::Matrix4d::Zero();
		H.block<3, 3>(0, 0) = R;
		H.block<3, 1>(0, 3) = R * d;
		H(3, 3) = 1.0;
		return H;
	};

	Eigen::Matrix4d make_HR_rotate_first(
	    const Eigen::Matrix3d& R,
	    const Eigen::Vector3d& d)
	{
		Eigen::Matrix4d H = Eigen::Matrix4d::Zero();
		H.block<3, 3>(0, 0) = R;
		H.block<3, 1>(0, 3) = d;
		H(3, 3) = 1.0;
		return H;
	};

	Eigen::Matrix4d make_HC_translate_first(
	    const Eigen::Matrix3d& C,
	    const Eigen::Vector3d& d)
	{
		Eigen::Matrix4d H = Eigen::Matrix4d::Zero();
		H.block<3, 3>(0, 0) = C;
		H.block<3, 1>(0, 3) = -C * d;
		H(3, 3) = 1.0;
		return H;
	}

	Eigen::Matrix4d make_HC_rotate_first(
	    const Eigen::Matrix3d& C,
	    const Eigen::Vector3d& d)
	{
		Eigen::Matrix4d H = Eigen::Matrix4d::Zero();
		H.block<3, 3>(0, 0) = C;
		H.block<3, 1>(0, 3) = -d;
		H(3, 3) = 1.0;
		return H;
	}

	// Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
	// Whether the net homogeneous transformation matrix represents an intrinsic or extrinsic rotation depends on whether the homogeneous transformation matrices passed to the function represent active rotations (ie vector rotations) or passive rotations (ie frame rotations)
	// Active: extrinsic -> pre-multiply; intrinsic -> not defined
	// Passive: extrinsic -> post-multiply; intrinsic -> pre-multiply
	// All vector rotations (as opposed to frame rotations) are, by definition, extrinsic. The concept of an "intrinsic" rotation only applies to frames
	// That is, the concept of an "intrinsic" vector rotation is not defined
	// Given the orientation (of the frame/vector) obtained via the nth transformation, how is the n+1 transformation applied
	Eigen::Matrix4d chain_hom_post(
	    const std::vector<Eigen::Matrix4d>& H_list)
	{
		Eigen::Matrix4d Htot = constants::HI;
		for (const auto& H : H_list) {
			Htot *= H;
		}
		return Htot;
	}

	// Given the orientation (of the frame/vector) obtained via the nth transformation, how is the n+1 transformation applied
	Eigen::Matrix4d chain_hom_pre(
	    const std::vector<Eigen::Matrix4d>& H_list)
	{
		Eigen::Matrix4d Htot = constants::HI;

		for (auto rit = H_list.rbegin(); rit != H_list.rend(); ++rit) {
			Htot *= *rit;
		}
		return Htot;
	}

} // namespace transforms
