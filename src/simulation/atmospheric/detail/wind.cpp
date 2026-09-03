#include "simulation/atmospheric/public/detail/wind.hpp"
#include "simulation/util/public/trig.hpp"
#include "simulation/util/public/units.hpp"

namespace atmospheric {

    Wind build_wind(double heading_deg, double spd_kts) {
        double psi_wind = util::deg_to_rad(heading_deg);
        double v_wind = util::kts_to_mps(spd_kts);

        double wind_n = -v_wind * util::cos(psi_wind);
        double wind_e = -v_wind * util::sin(psi_wind);
        double wind_d = 0.0;

        return { Eigen::Vector3d(wind_n, wind_e, wind_d) };
    }

}
