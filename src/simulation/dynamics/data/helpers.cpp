#include <Eigen/Dense>
#include "simulation/dynamics/public/data/helpers.hpp"

namespace dynamics {

	WrenchVector_T<double> unpack_wrench(
	    const Wrench& wrench)
	{
		WrenchVector_T<double> out;
		out << wrench.F.data, wrench.M.data;
		return out;
	}

	Wrench pack_wrench(
	    const WrenchVector_T<double>& wrench)
	{
		return {.F = Force{Eigen::Vector3d(wrench(0), wrench(1), wrench(2))},
		    .M = Moment{Eigen::Vector3d(wrench(3), wrench(4), wrench(5))}};
	}

	Wrench pack_wrench(
	    const Wrench_T<double>& wrench)
	{
		return {
		    .F = dynamics::Force{wrench.F},
		    .M = dynamics::Moment{wrench.M},
		};
	}

	State_T<double> pack_state(
	    const RigidBodyState& Xt)
	{
		TranslationalVelocity vB_BI = Xt.v;
		AngularVelocity wB_BI = Xt.w;
		EulerAngles eulIB;
		eulIB.set(Xt.q);

		return {
		    .vx = vB_BI.data(0),
		    .vy = vB_BI.data(1),
		    .vz = vB_BI.data(2),
		    .p = wB_BI.p(),
		    .q = wB_BI.q(),
		    .r = wB_BI.r(),
		    .phi = eulIB.phi(),
		    .theta = eulIB.theta(),
		};
	}

	StateVector_T<double> unpack_state(
	    const RigidBodyState& Xt)
	{
		return unpack_state_T(pack_state(Xt));
	}

} // namespace dynamics
