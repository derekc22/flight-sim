#include "simulation/global/global.hpp"

namespace global {

    // Constants
    const double r_earth = 6.371e6; // [m]
    const double gravity = 9.81;    // [ms^-2]
    const double pi = 3.1415926535897932384626433832795028841971693993751;
    const double eps = 1e-12;
    const double dt = 0.001;        // [s]

    // Special Matrices
    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    const Eigen::Matrix4d I4 = Eigen::Matrix4d::Identity();
    const Eigen::Matrix4d HI = Eigen::Matrix4d::Identity(); // Identity homogenous transformation matrix
    const Eigen::Matrix3d Zero3x3 = Eigen::Matrix3d::Zero();

    // Special Vectors
    const Eigen::Vector3d Zero3 = Eigen::Vector3d::Zero();
    const Eigen::Vector3d i = Eigen::Vector3d(1, 0, 0);
    const Eigen::Vector3d j = Eigen::Vector3d(0, 1, 0);
    const Eigen::Vector3d k = Eigen::Vector3d(0, 0, 1);


    // Trigonometry
    double deg_to_rad(double deg){
        return deg * (pi / 180.0);
    };

    double rad_to_deg(double rad){
        return rad * (180.0 / pi);
    };

    double sec(double theta) {
        return 1.0 / std::cos(theta);
    }

    double csc(double theta) {
        return 1.0 / std::sin(theta);
    }

    double cot(double theta) {
        return 1.0 / std::tan(theta);
    }

    double kts_to_ms(double kts) {
        return kts/1.94384;
    }

    double ms_to_kts(double ms) {
        return ms * 1.94384;
    }

    double ft_to_m(double ft){
        return ft/3.281;
    }

    double m_to_ft(double m){
        return m * 3.281;
    }


    // Linear Algebra Operators
    Eigen::Matrix3d hat(const Eigen::Vector3d& v){
        Eigen::Matrix3d S;
        S <<     0, -v.z(),   v.y(),
             v.z(),      0,  -v.x(),
            -v.y(),  v.x(),       0;
        return S;
    }

    Eigen::Vector3d vee(const Eigen::Matrix3d& S){
        return Eigen::Vector3d(
            S(2,1),
            S(0,2),
            S(1,0)
        );
    }


    Eigen::Vector3d norm(const Eigen::Vector3d& v) {
        const double n = v.norm();
        if (n < global::eps) { return global::Zero3; }
        return v / n;
    }

    double clamp_symmetric(double x, double max_abs) {
        if (max_abs <= 0.0) return 0.0;
        return std::clamp(x, -max_abs, max_abs);
    }

    double clamp_positive(double x, double max_value) {
        if (max_value <= 0.0) return 0.0;
        return std::clamp(x, 0.0, max_value);
    }

    double clamp_to_1(double x) {
        // clamps to [-1, 1]
        return std::clamp(x, -1.0, 1.0);
    }

    double wrap_to_pi(double x) {
        // maps to (-pi, pi]
        return std::remainder(x, 2.0 * ::global::pi);
    }

    double sin(double x) {
        return std::sin(x);
    }

    double cos(double x) {
        return std::cos(x);
    }

    double tan(double x) {
        return std::tan(x);
    }

    double asin(double x) {
        return std::asin(x);
    }

    double atan2(double y, double x) {
        return std::atan2(y, x);
    }

    double sqrt(double x) {
        return std::sqrt(x);
    }

    double abs(double x) {
        return std::abs(x);
    }

    CppAD::AD<double> sin(const CppAD::AD<double>& x) {
        return CppAD::sin(x);
    }

    CppAD::AD<double> cos(const CppAD::AD<double>& x) {
        return CppAD::cos(x);
    }

    CppAD::AD<double> tan(const CppAD::AD<double>& x) {
        return CppAD::tan(x);
    }

    CppAD::AD<double> asin(const CppAD::AD<double>& x) {
        return CppAD::asin(x);
    }

    CppAD::AD<double> atan2(const CppAD::AD<double>& y, const CppAD::AD<double>& x) {
        return CppAD::atan2(y, x);
    }

    CppAD::AD<double> sqrt(const CppAD::AD<double>& x) {
        return CppAD::sqrt(x);
    }

    CppAD::AD<double> abs(const CppAD::AD<double>& x) {
        return CppAD::abs(x);
    }

    CppAD::AD<double> clamp_symmetric(const CppAD::AD<double>& x, double max_abs) {
        if (max_abs <= 0.0) return CppAD::AD<double>(0.0);
        const CppAD::AD<double> max_t(max_abs);
        if (x > max_t) return max_t;
        if (x < -max_t) return -max_t;
        return x;
    }

    CppAD::AD<double> clamp_positive(const CppAD::AD<double>& x, double max_value) {
        if (max_value <= 0.0) return CppAD::AD<double>(0.0);
        const CppAD::AD<double> max_t(max_value);
        if (x < CppAD::AD<double>(0.0)) return CppAD::AD<double>(0.0);
        if (x > max_t) return max_t;
        return x;
    }

    CppAD::AD<double> clamp_to_1(const CppAD::AD<double>& x) {
        if (x > CppAD::AD<double>(1.0)) return CppAD::AD<double>(1.0);
        if (x < CppAD::AD<double>(-1.0)) return CppAD::AD<double>(-1.0);
        return x;
    }

    double vector_norm(const Eigen::Vector3d& v) {
        return sqrt(v.dot(v));
    }

    CppAD::AD<double> vector_norm(const Eigen::Matrix<CppAD::AD<double>, 3, 1>& v) {
        return sqrt(v.dot(v));
    }

    Eigen::Matrix<CppAD::AD<double>, 3, 1> norm(const Eigen::Matrix<CppAD::AD<double>, 3, 1>& v) {
        const CppAD::AD<double> n = vector_norm(v);
        if (n < CppAD::AD<double>(global::eps)) {
            return Eigen::Matrix<CppAD::AD<double>, 3, 1>::Zero();
        }
        return v / n;
    }

}
