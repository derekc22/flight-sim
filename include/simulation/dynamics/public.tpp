#pragma once
#include "simulation/util/public.hpp"

namespace dynamics {

    template <typename T>
    StateVector_T<T> unpack_state_T(const State_T<T>& x) {
        StateVector_T<T> out;
        out << x.vx, x.vy, x.vz,
               x.p, x.q, x.r,
               x.phi, x.theta;
        return out;
    }

    template <typename T>
    StateDotVector_T<T> unpack_state_dot_T(const StateDot_T<T>& x_dot) {
        StateDotVector_T<T> out;
        out << x_dot.vx_dot, x_dot.vy_dot, x_dot.vz_dot,
               x_dot.p_dot, x_dot.q_dot, x_dot.r_dot,
               x_dot.phi_dot, x_dot.theta_dot;
        return out;
    }

    template <typename T>
    constants::Vector3_T<T> ddtB_vB_BI_T(const constants::Vector3_T<T>& vB, const constants::Vector3_T<T>& wB_BI, double mass, const constants::Vector3_T<T>& FB_net) {
        return (FB_net / T(mass)) - wB_BI.cross(vB);
    }

    template <typename T>
    constants::Vector3_T<T> ddtB_wB_BI_T(const constants::Vector3_T<T>& wB_BI, const Eigen::Matrix3d& J, const constants::Vector3_T<T>& MB_net) {
        const constants::Matrix3_T<T> J_T = J.template cast<T>();
        const constants::Matrix3_T<T> J_inv_T = J.inverse().template cast<T>();
        const constants::Vector3_T<T> rhs = MB_net - wB_BI.cross(J_T * wB_BI);
        return J_inv_T * rhs;
    }

    template <typename T>
    constants::Matrix3_T<T> wB_BI_to_eul_dot_mat_T(const T& theta, const T& phi) {
        const T sec_theta = T(1) / util::cos(theta);
        constants::Matrix3_T<T> T_mat;
        T_mat << T(1), util::sin(phi) * util::tan(theta), util::cos(phi) * util::tan(theta),
                 T(0), util::cos(phi),                      -util::sin(phi),
                 T(0), util::sin(phi) * sec_theta,          util::cos(phi) * sec_theta;
        return T_mat;
    }

    template <typename T>
    constants::Vector3_T<T> wB_BI_to_eul_dot_T(const constants::Vector3_T<T>& wB_BI, const T& theta, const T& phi) {
        return wB_BI_to_eul_dot_mat_T<T>(theta, phi) * wB_BI;
    }

    template <typename T>
    constants::Matrix3_T<T> eul_dot_to_wB_BI_mat_T(const T& theta, const T& phi) {
        constants::Matrix3_T<T> T_mat;
        T_mat << T(1), T(0), -util::sin(theta),
                 T(0), util::cos(phi), util::sin(phi) * util::cos(theta),
                 T(0), -util::sin(phi), util::cos(phi) * util::cos(theta);
        return T_mat;
    }

    template <typename T>
    constants::Vector3_T<T> eul_dot_to_wB_BI_T(const constants::Vector3_T<T>& eul_dot, const T& theta, const T& phi) {
        return eul_dot_to_wB_BI_mat_T<T>(theta, phi) * eul_dot;
    }

    template <typename T>
    constants::Vector3_T<T> ddtB_to_ddtI_T(const constants::Vector3_T<T>& ddtB_v, const constants::Vector3_T<T>& v, const constants::Vector3_T<T>& w) {
        return ddtB_v + w.cross(v);
    }
}
