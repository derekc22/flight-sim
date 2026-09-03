#include <Eigen/Core>
#include "simulation/aerodynamics/public/data/helpers.hpp"
#include "simulation/util/public/trig.hpp"

namespace aerodynamics {

    dynamics::OrientationMatrix CBS(const AngleOfAttack& alpha) {
        Eigen::Matrix3d CBS;
        double a = alpha.data;
        CBS     <<   util::cos(a),   0,   util::sin(a),
                                0,   1,              0,
                    -util::sin(a),   0,   util::cos(a);
        return { CBS };
    };

    dynamics::OrientationMatrix CSW(const SideslipAngle& beta) {
        Eigen::Matrix3d CSW;
        double b = beta.data;
        CSW     <<   util::cos(b),   util::sin(b),   0,
                    -util::sin(b),   util::cos(b),   0,
                                0,              0,   1;
        return { CSW };
    };

}
