#pragma once

namespace dynamics {
    template <typename T>
    Matrix3_T<T> _eul_dot_to_wB_BI_mat_T(const T& theta, const T& phi) {
        Matrix3_T<T> T_mat;
        T_mat << T(1), T(0), -global::sin(theta),
                 T(0), global::cos(phi), global::sin(phi) * global::cos(theta),
                 T(0), -global::sin(phi), global::cos(phi) * global::cos(theta);
        return T_mat;
    }

    template <typename T>
    Matrix3_T<T> _wB_BI_to_eul_dot_mat_T(const T& theta, const T& phi) {
        const T sec_theta = T(1) / global::cos(theta);
        Matrix3_T<T> T_mat;
        T_mat << T(1), global::sin(phi) * global::tan(theta), global::cos(phi) * global::tan(theta),
                 T(0), global::cos(phi),                      -global::sin(phi),
                 T(0), global::sin(phi) * sec_theta,          global::cos(phi) * sec_theta;
        return T_mat;
    }

    template <typename T>
    Vector3_T<T> _ddtB_vB_BI_T(const Vector3_T<T>& vB, const Vector3_T<T>& wB_BI, double mass, const Vector3_T<T>& FB_net) {
        return (FB_net / T(mass)) - wB_BI.cross(vB);
    }

    template <typename T>
    Vector3_T<T> _ddtB_wB_BI_T(const Vector3_T<T>& wB_BI, const Eigen::Matrix3d& J, const Vector3_T<T>& MB_net) {
        const Matrix3_T<T> J_T = J.template cast<T>();
        const Matrix3_T<T> J_inv_T = J.inverse().template cast<T>();
        const Vector3_T<T> rhs = MB_net - wB_BI.cross(J_T * wB_BI);
        return J_inv_T * rhs;
    }

    template <typename T>
    Vector3_T<T> _ddtB_to_ddtI_T(const Vector3_T<T>& ddtB_v, const Vector3_T<T>& v, const Vector3_T<T>& w) {
        return ddtB_v + w.cross(v);
    }

    template <typename T>
    Vector3_T<T> _eul_dot2wB_BI_T(const Vector3_T<T>& eul_dot, const T& theta, const T& phi) {
        return _eul_dot_to_wB_BI_mat_T(theta, phi) * eul_dot;
    }

    template <typename T>
    Vector3_T<T> _wB_BI_to_eul_dot_T(const Vector3_T<T>& wB_BI, const T& theta, const T& phi) {
        return _wB_BI_to_eul_dot_mat_T(theta, phi) * wB_BI;
    }
}
