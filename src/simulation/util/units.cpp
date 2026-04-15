#include "simulation/util/units.hpp"

namespace util {

    double kts_to_ms(double kts) {
        return kts / 1.94384;
    }

    double ms_to_kts(double ms) {
        return ms * 1.94384;
    }

    double ft_to_m(double ft) {
        return ft / 3.281;
    }

    double m_to_ft(double m) {
        return m * 3.281;
    }

}
