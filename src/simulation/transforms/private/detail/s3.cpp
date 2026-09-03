#include <Eigen/Dense>
#include <stdexcept>
#include <algorithm>
#include <string>
#include <vector>
#include "simulation/constants/public/linalg.hpp"
#include "simulation/transforms/private/detail/s3.hpp"
#include "simulation/transforms/private/detail/so3.hpp"
#include "simulation/transforms/public/detail/s3.hpp"
#include "simulation/util/public/trig.hpp"

namespace transforms {

	/** @deprecated */
	// Eigen::Quaterniond quat_mul(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
	//     double w1 = q1.w(), x1 = q1.x(), y1 = q1.y(), z1 = q1.z();
	//     double w2 = q2.w(), x2 = q2.x(), y2 = q2.y(), z2 = q2.z();

	//     Eigen::Quaterniond q;
	//     q.w() = w1*w2 - x1*x2 - y1*y2 - z1*z2;
	//     q.x() = w1*x2 + x1*w2 + y1*z2 - z1*y2;
	//     q.y() = w1*y2 - x1*z2 + y1*w2 + z1*x2;
	//     q.z() = w1*z2 + x1*y2 - y1*x2 + z1*w2;
	//     return q;
	// }

	Eigen::Quaterniond qx(
	    double phi)
	{
		double h = 0.5 * phi;
		return Eigen::Quaterniond(util::cos(h), util::sin(h), 0.0, 0.0); // (w,x,y,z)
	}

	Eigen::Quaterniond qy(
	    double theta)
	{
		double h = 0.5 * theta;
		return Eigen::Quaterniond(util::cos(h), 0.0, util::sin(h), 0.0);
	}

	Eigen::Quaterniond qz(
	    double psi)
	{
		double h = 0.5 * psi;
		return Eigen::Quaterniond(util::cos(h), 0.0, 0.0, util::sin(h));
	}

	Eigen::Quaterniond eul_to_quatR_extr(
	    double a,
	    double b,
	    double c,
	    EulerOrder order)
	{
		Eigen::Quaterniond q;

		switch (order) {
			case EulerOrder::ZYX:
				q = qx(c) * qy(b) * qz(a);
				break;
			case EulerOrder::ZXY:
				q = qy(c) * qx(b) * qz(a);
				break;
			case EulerOrder::YZX:
				q = qx(c) * qz(b) * qy(a);
				break;
			case EulerOrder::YXZ:
				q = qz(c) * qx(b) * qy(a);
				break;
			case EulerOrder::XZY:
				q = qy(c) * qz(b) * qx(a);
				break;
			case EulerOrder::XYZ:
				q = qz(c) * qy(b) * qx(a);
				break;

			// Proper Euler (repeated axis)
			case EulerOrder::ZYZ:
				q = qz(c) * qy(b) * qz(a);
				break;
			case EulerOrder::ZXZ:
				q = qz(c) * qx(b) * qz(a);
				break;
			case EulerOrder::XZX:
				q = qx(c) * qz(b) * qx(a);
				break;
			case EulerOrder::XYX:
				q = qx(c) * qy(b) * qx(a);
				break;
			case EulerOrder::YZY:
				q = qy(c) * qz(b) * qy(a);
				break;
			case EulerOrder::YXY:
				q = qy(c) * qx(b) * qy(a);
				break;

			default:
				throw std::invalid_argument("Unsupported Euler order");
		}

		return normalize_and_canonicalize(q);
	}

	// All vector rotations (as opposed to frame rotations) are, by definition, extrinsic. The concept of an "intrinsic" rotation only applies to frames
	// That is, the concept of an "intrinsic" vector rotation is not defined
	// So, as with eul_to_R_intr, the function 'eul_to_quatR_intr' does not technically make sense
	// However, for consistency with the extrinsic case, it is still implemented since transposing the result of this function indeed gives the correct result for an intrinsic frame rotation/coordinate transformation
	Eigen::Quaterniond eul_to_quatR_intr(
	    double a,
	    double b,
	    double c,
	    EulerOrder order)
	{
		return normalize_and_canonicalize(eul_to_quatR_extr(-a, -b, -c, order).conjugate());
	}

	Eigen::Quaterniond eul_to_quatC_extr(
	    double a,
	    double b,
	    double c,
	    EulerOrder order)
	{
		return normalize_and_canonicalize(eul_to_quatR_extr(a, b, c, order).conjugate());
	}

	Eigen::Quaterniond eul_to_quatC_intr(
	    double a,
	    double b,
	    double c,
	    EulerOrder order)
	{
		return normalize_and_canonicalize(eul_to_quatR_intr(a, b, c, order).conjugate());
	}

	Eigen::Vector3d quatR_to_eul_intr(
	    const Eigen::Quaterniond& q,
	    EulerOrder order)
	{
		Eigen::Matrix3d R = quat_to_rot(q);
		return R_to_eul_intr(R, order);
	}

	Eigen::Vector3d quatC_to_eul_intr(
	    const Eigen::Quaterniond& q,
	    EulerOrder order)
	{
		Eigen::Matrix3d C = quat_to_rot(q);
		return C_to_eul_intr(C, order);
	}

	Eigen::Vector3d quatR_to_eul_extr(
	    const Eigen::Quaterniond& q,
	    EulerOrder order)
	{
		Eigen::Matrix3d R = quat_to_rot(q);
		return R_to_eul_extr(R, order);
	}

	Eigen::Vector3d quatC_to_eul_extr(
	    const Eigen::Quaterniond& q,
	    EulerOrder order)
	{
		Eigen::Matrix3d C = quat_to_rot(q);
		return C_to_eul_extr(C, order);
	}

	// Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
	// Whether the net quaternion represents an intrinsic or extrinsic rotation depends on whether the quaternions passed to the function represent active rotations (ie vector rotations) or passive rotations (ie frame rotations)
	// Active: extrinsic -> pre-multiply; intrinsic -> not defined
	// Passive: extrinsic -> post-multiply; intrinsic -> pre-multiply
	// All vector rotations (as opposed to frame rotations) are, by definition, extrinsic. The concept of an "intrinsic" rotation only applies to frames
	// That is, the concept of an "intrinsic" vector rotation is not defined
	Eigen::Quaterniond chain_quat_post(
	    const std::vector<Eigen::Quaterniond>& q_list)
	{
		Eigen::Quaterniond qtot = constants::qI;
		for (const auto& q : q_list) {
			qtot *= normalize_and_canonicalize(q);
		}
		return normalize_and_canonicalize(qtot);
	}

	// Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
	Eigen::Quaterniond chain_quat_pre(
	    const std::vector<Eigen::Quaterniond>& q_list)
	{
		Eigen::Quaterniond qtot = constants::qI;

		for (auto rit = q_list.rbegin(); rit != q_list.rend(); ++rit) {
			qtot *= normalize_and_canonicalize(*rit);
		}
		return normalize_and_canonicalize(qtot);
	}

} // namespace transforms
