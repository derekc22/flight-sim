#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include <string>
#include <tuple> // For std::tuple
#include <array>
#include <functional>
#include "simulation/constants/constants.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/util/util.hpp"
#include "simulation/transforms/transforms.hpp"

namespace frames { struct Frame; } // forward declare

namespace dynamics {

    /** @warning The parent of F must be an inertial frame: ECEFFrame or NEDFrameECEF */
    RigidBodyState compute_rigid_body_state(const frames::Frame& F);

    template <typename T>
    constants::Matrix3_T<T> _eul_dot_to_wB_BI_mat_T(const T& theta, const T& phi);

    template <typename T>
    constants::Matrix3_T<T> _wB_BI_to_eul_dot_mat_T(const T& theta, const T& phi);

    template <typename T>
    constants::Vector3_T<T> _ddtB_vB_BI_T(const constants::Vector3_T<T>& vB, const constants::Vector3_T<T>& wB_BI, double mass, const constants::Vector3_T<T>& FB_net);

    template <typename T>
    constants::Vector3_T<T> _ddtB_wB_BI_T(const constants::Vector3_T<T>& wB_BI, const Eigen::Matrix3d& J, const constants::Vector3_T<T>& MB_net);

    template <typename T>
    constants::Vector3_T<T> _ddtB_to_ddtI_T(const constants::Vector3_T<T>& ddtB_v, const constants::Vector3_T<T>& v, const constants::Vector3_T<T>& w);

    template <typename T>
    constants::Vector3_T<T> _eul_dot_to_wB_BI_T(const constants::Vector3_T<T>& eul_dot, const T& theta, const T& phi);

    template <typename T>
    constants::Vector3_T<T> _wB_BI_to_eul_dot_T(const constants::Vector3_T<T>& wB_BI, const T& theta, const T& phi);



    /** @warning Function signatures with an 'I' indicate that arguments MUST be specified WRT an inertial frame
        In this codebase, 'inertial' is used as a strict synonym for the ECEF frame
        However, as implemented in this codebase, the NED frame is technically also an inertial frame, so the NED frame could technically also be used with these functions
        This is because, as implemented in this codebase, the NED frame remains fixed for the duration of the flight - as opposed to translating/re-orienting as the aircraft moves (which would be non-inertial behavior)
        For simplicity, however, we strictly assume that 'I' and 'inertial' refer to the ECEF frame in the below dynamics functions
        This may change in the future, but, for now, always assume inertial <=> ECEF and NEVER pass arguments corresponding to other frames to the below dynamics functions
    */

    EulerAngleRates _wB_BI_to_eul_dot(const AngularVelocity& wB_BI, const EulerAngles& eul);
    Position _trans_kin(const Position& xt, const LinearVelocity& xt_dot, const LinearAcceleration& xt_ddot);
    LinearVelocity _trans_kin_vel(const LinearVelocity& xt_dot, const LinearAcceleration& xt_ddot);
    EulerAngles _eul_kin(const EulerAngles& eul_t, const EulerAngleRates& eul_dot_t);

    OrientationMatrixRate _ddt_CIB(const OrientationMatrix& CIB, const AngularVelocity& wB_BI);
    OrientationMatrixRate _ddt_CBI(const OrientationMatrix& CBI, const AngularVelocity& wB_BI);
    OrientationMatrix _rot_kin(const OrientationMatrix& CIB_t, const AngularVelocity& wB_BI_t);
    OrientationQuaternion _quat_kin(const OrientationQuaternion& qIB_t, const AngularVelocity& wB_BI_t);
    OrientationQuaternionRate _quat_kin_vel(const OrientationQuaternion& qIB_t, const AngularVelocity& wB_BI_t);

    /**
    * @brief Returns the body derivative of body-expressed linear velocity
    */
    LinearAcceleration _ddtB_vB_BI(const LinearVelocity& vB, const AngularVelocity& wB_BI, const Mass& mass, const Force& FB_net);
    
    /**
    * @brief Returns the body derivative of body-expressed angular velocity
    */
    Eigen::Vector3d _ddtB_wB_BI(const AngularVelocity& wB_BI, const InertiaTensor& J, const Moment& MB_net);


    /**
    * @brief Converts a body derivative to an inertial derivative
    */
    Eigen::Vector3d _ddtB_to_ddtI(const Eigen::Vector3d& ddtB_v, const Eigen::Vector3d& v, const Eigen::Vector3d& w);


    LinearVelocity _trans_dyn_vel(const LinearVelocity& vB_t, const AngularVelocity& wB_BI_t, const Mass& mass, const Force& FB_net_t);
    AngularVelocity _rot_dyn(const AngularVelocity& wB_BI_t, const InertiaTensor& J, const Moment& MB_net_t);
    RigidBodyState step_rigid_body(const RigidBodyState& xB_BI_t, const Mass& mass, const InertiaTensor& J, const Wrench& WB_net_t);

    AngularVelocity _CIB_dot_to_wB_BI(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB);
    OrientationQuaternionRate _CIB_dot_to_qIB_dot(const OrientationMatrixRate& CIB_dot, const OrientationMatrix& CIB, const OrientationQuaternion& qIB);
    AngularVelocity _qIB_dot_to_wB_BI(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB);
    OrientationMatrixRate _qIB_dot_to_CIB_dot(const OrientationQuaternionRate& qIB_dot, const OrientationQuaternion& qIB, const OrientationMatrix& CIB);
    AngularVelocity _eul_dot_to_wB_BI(const EulerAngleRates& eul_dot, const EulerAngles& eul);
    EulerAngleRates _wB_BI_to_eul_dot(const AngularVelocity& wB_BI, const EulerAngles& eul);

    Eigen::Matrix3d _eul_dot_to_wB_BI_mat(double theta, double phi);

    Eigen::Matrix3d _wB_BI_to_eul_dot_mat(double theta, double phi);
}

#include "simulation/dynamics/dynamics.tpp"
