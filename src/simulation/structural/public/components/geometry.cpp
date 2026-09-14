#include "simulation/structural/public/components/geometry.hpp"

#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/util/public/linalg.hpp"

#include <stdexcept>

namespace structural
{

	Eigen::Matrix3d Geometry::compute_local_J()
	{
		// Moments of inertia of rectangular prism about its own center, expressed in its local frame
		Eigen::Matrix3d j = constants::Zero3x3;
		j(0, 0) = (1.0 / 12.0) * mass * (y_size * y_size + z_size * z_size); // Jxx
		j(1, 1) = (1.0 / 12.0) * mass * (x_size * x_size + z_size * z_size); // Jyy
		j(2, 2) = (1.0 / 12.0) * mass * (x_size * x_size + y_size * y_size); // Jzz
		return j;
	}

	double Geometry::compute_spin_inertia(
		const Eigen::Vector3d& axisB)
	{
		if (axisB.norm() < constants::eps) {
			throw std::runtime_error("structural::Geometry::compute_spin_inertia: spin axis cannot be zero");
		}
		Eigen::Vector3d axisL = CBL * axisB.normalized();
		return axisL.dot(compute_local_J() * axisL);
	}

} // namespace structural
