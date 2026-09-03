#include <Eigen/Dense>
#include "simulation/constants/public/scalars.hpp"
#include "simulation/geography/private/detail/gravity.hpp"
#include "simulation/geography/public/detail/gravity.hpp"
#include "simulation/transforms/public/detail/so3.hpp"

namespace geography {

	dynamics::Gravity gN()
	{
		return {Eigen::Vector3d(0.0, 0.0, constants::g_earth)};
	};

	dynamics::Gravity gB(
	    const dynamics::Position& pE,
	    const dynamics::HomogeneousTransformationMatrix& HEB)
	{
		return {HEB.C().data * gE(HEB.p()).data};
	};

	dynamics::Gravity gB(
	    const dynamics::Position& pE,
	    const dynamics::OrientationMatrix& CEB)
	{
		return {CEB.data * gE(pE).data};
	};

	dynamics::Gravity gB(
	    const dynamics::Position& pE,
	    const dynamics::OrientationQuaternion& qEB)
	{
		return {qEB.data * gE(pE).data};
	};

	dynamics::Gravity gB(
	    const dynamics::Position& pE,
	    const dynamics::EulerAngles& eulEB)
	{
		return {transforms::eul_to_C(eulEB.psi(),
		            eulEB.theta(),
		            eulEB.phi(),
		            transforms::EulerOrder::ZYX,
		            transforms::RotationType::Intrinsic) *
		    gE(pE).data};
	};

	dynamics::Gravity gB(
	    const dynamics::HomogeneousTransformationMatrix& HNB)
	{
		return {HNB.C().data * gN().data};
	};

	dynamics::Gravity gB(
	    const dynamics::OrientationMatrix& CNB)
	{
		return {CNB.data * gN().data};
	};

	dynamics::Gravity gB(
	    const dynamics::OrientationQuaternion& qNB)
	{
		return {qNB.data * gN().data};
	};

	dynamics::Gravity gB(
	    const dynamics::EulerAngles& eulNB)
	{
		return {transforms::eul_to_C(eulNB.psi(),
		            eulNB.theta(),
		            eulNB.phi(),
		            transforms::EulerOrder::ZYX,
		            transforms::RotationType::Intrinsic) *
		    gN().data};
	};

	dynamics::Gravity gS(
	    const dynamics::Gravity& gB,
	    const dynamics::OrientationMatrix& CBS)
	{
		return {CBS.data * gB.data};
	};

	dynamics::Gravity gW(
	    const dynamics::Gravity& gS,
	    const dynamics::OrientationMatrix& CSW)
	{
		return {CSW.data * gS.data};
	};

} // namespace geography
