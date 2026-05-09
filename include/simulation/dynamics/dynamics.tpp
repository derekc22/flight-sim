#pragma once
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"

namespace dynamics {
    template <typename T>
    constants::Matrix3_T<T> _eul_dot_to_wB_BI_mat_T(const T& theta, const T& phi) {
        constants::Matrix3_T<T> T_mat;
        T_mat << T(1), T(0), -util::sin(theta),
                 T(0), util::cos(phi), util::sin(phi) * util::cos(theta),
                 T(0), -util::sin(phi), util::cos(phi) * util::cos(theta);
        return T_mat;
    }

    template <typename T>
    constants::Matrix3_T<T> _wB_BI_to_eul_dot_mat_T(const T& theta, const T& phi) {
        const T sec_theta = T(1) / util::cos(theta);
        constants::Matrix3_T<T> T_mat;
        T_mat << T(1), util::sin(phi) * util::tan(theta), util::cos(phi) * util::tan(theta),
                 T(0), util::cos(phi),                      -util::sin(phi),
                 T(0), util::sin(phi) * sec_theta,          util::cos(phi) * sec_theta;
        return T_mat;
    }

    template <typename T>
    constants::Vector3_T<T> _ddtB_vB_BI_T(const constants::Vector3_T<T>& vB, const constants::Vector3_T<T>& wB_BI, double mass, const constants::Vector3_T<T>& FB_net) {
        return (FB_net / T(mass)) - wB_BI.cross(vB);
    }

    template <typename T>
    constants::Vector3_T<T> _ddtB_wB_BI_T(const constants::Vector3_T<T>& wB_BI, const Eigen::Matrix3d& J, const constants::Vector3_T<T>& MB_net) {
        const constants::Matrix3_T<T> J_T = J.template cast<T>();
        const constants::Matrix3_T<T> J_inv_T = J.inverse().template cast<T>();
        const constants::Vector3_T<T> rhs = MB_net - wB_BI.cross(J_T * wB_BI);
        return J_inv_T * rhs;
    }

    template <typename T>
    constants::Vector3_T<T> _ddtB_to_ddtI_T(const constants::Vector3_T<T>& ddtB_v, const constants::Vector3_T<T>& v, const constants::Vector3_T<T>& w) {
        return ddtB_v + w.cross(v);
    }

    template <typename T>
    constants::Vector3_T<T> _eul_dot_to_wB_BI_T(const constants::Vector3_T<T>& eul_dot, const T& theta, const T& phi) {
        return _eul_dot_to_wB_BI_mat_T(theta, phi) * eul_dot;
    }

    template <typename T>
    constants::Vector3_T<T> _wB_BI_to_eul_dot_T(const constants::Vector3_T<T>& wB_BI, const T& theta, const T& phi) {
        return _wB_BI_to_eul_dot_mat_T(theta, phi) * wB_BI;
    }
}
