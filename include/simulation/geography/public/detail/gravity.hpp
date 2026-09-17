#pragma once
#include "simulation/constants/public/linalg.hpp"
#include "simulation/dynamics/public/data/types.hpp"

namespace geography
{

	/**
	 * @brief Returns the constant gravity vector expressed in NED axes.
	 *
	 * @return NED gravity vector with positive down component [m/s^2].
	 */
	dynamics::Gravity gN();

	/**
	 * @brief Computes gravity expressed in body axes from an ECEF-to-body transformation.
	 *
	 * Uses the position and orientation contained in @p HEB.
	 *
	 * @param[in] HEB ECEF-to-body homogeneous transformation.
	 * @return Gravity vector expressed in body axes [m/s^2].
	 */
	dynamics::Gravity gB_from_E(const dynamics::HomogeneousTransformationMatrix& HEB);

	/**
	 * @brief Computes gravity expressed in body axes from ECEF position and orientation.
	 *
	 * @param[in] pE Position relative to Earth center expressed in ECEF axes [m].
	 * @param[in] CEB ECEF-to-body orientation matrix [-].
	 * @return Gravity vector expressed in body axes [m/s^2].
	 */
	dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::OrientationMatrix& CEB);

	/**
	 * @brief Computes gravity expressed in body axes from ECEF position and orientation.
	 *
	 * @param[in] pE Position relative to Earth center expressed in ECEF axes [m].
	 * @param[in] qEB ECEF-to-body orientation quaternion [-].
	 * @return Gravity vector expressed in body axes [m/s^2].
	 */
	dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::OrientationQuaternion& qEB);

	/**
	 * @brief Computes gravity expressed in body axes from ECEF position and Euler angles.
	 *
	 * @param[in] pE Position relative to Earth center expressed in ECEF axes [m].
	 * @param[in] eulEB Intrinsic ZYX Euler angles defining the ECEF-to-body orientation [rad].
	 * @return Gravity vector expressed in body axes [m/s^2].
	 */
	dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::EulerAngles& eulEB);

	/**
	 * @brief Expresses NED gravity in body axes using a NED-to-body transformation.
	 *
	 * @param[in] HNB NED-to-body homogeneous transformation.
	 * @return Gravity vector expressed in body axes [m/s^2].
	 */
	dynamics::Gravity gB_from_N(const dynamics::HomogeneousTransformationMatrix& HNB);

	/**
	 * @brief Expresses NED gravity in body axes using an orientation matrix.
	 *
	 * @param[in] CNB NED-to-body orientation matrix [-].
	 * @return Gravity vector expressed in body axes [m/s^2].
	 */
	dynamics::Gravity gB(const dynamics::OrientationMatrix& CNB);

	/**
	 * @brief Expresses NED gravity in body axes using an orientation quaternion.
	 *
	 * @param[in] qNB NED-to-body orientation quaternion [-].
	 * @return Gravity vector expressed in body axes [m/s^2].
	 */
	dynamics::Gravity gB(const dynamics::OrientationQuaternion& qNB);

	/**
	 * @brief Expresses NED gravity in body axes using Euler angles.
	 *
	 * @param[in] eulNB Intrinsic ZYX Euler angles defining the NED-to-body orientation [rad].
	 * @return Gravity vector expressed in body axes [m/s^2].
	 */
	dynamics::Gravity gB(const dynamics::EulerAngles& eulNB);

	/**
	 * @brief Expresses a body-frame gravity vector in stability axes.
	 *
	 * @param[in] gB Gravity vector expressed in body axes [m/s^2].
	 * @param[in] CBS Body-to-stability orientation matrix [-].
	 * @return Gravity vector expressed in stability axes [m/s^2].
	 */
	dynamics::Gravity gS(const dynamics::Gravity& gB, const dynamics::OrientationMatrix& CBS);

	/**
	 * @brief Expresses a stability-frame gravity vector in wind axes.
	 *
	 * @param[in] gS Gravity vector expressed in stability axes [m/s^2].
	 * @param[in] CSW Stability-to-wind orientation matrix [-].
	 * @return Gravity vector expressed in wind axes [m/s^2].
	 */
	dynamics::Gravity gW(const dynamics::Gravity& gS, const dynamics::OrientationMatrix& CSW);

	/**
	 * @brief Computes constant-magnitude gravity expressed in body axes from roll and pitch.
	 *
	 * @tparam T Scalar type used for the calculation.
	 * @param[in] phi Roll angle [rad].
	 * @param[in] theta Pitch angle [rad].
	 * @return Gravity vector expressed in body axes [m/s^2].
	 */
	template <typename T> constants::Vector3_T<T> gB_T(const T& phi, const T& theta);

} // namespace geography

#include "simulation/geography/public/detail/gravity.tpp"
