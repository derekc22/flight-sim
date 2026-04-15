#include <cmath>
#include "simulation/constants/constants.hpp"
#include "simulation/util/trig.hpp"

namespace util {

    double deg_to_rad(double deg) {
        return deg * (constants::pi / 180.0);
    }

    double rad_to_deg(double rad) {
        return rad * (180.0 / constants::pi);
    }

    double sec(double theta) {
        return 1.0 / std::cos(theta);
    }

    double csc(double theta) {
        return 1.0 / std::sin(theta);
    }

    double cot(double theta) {
        return 1.0 / std::tan(theta);
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

}
