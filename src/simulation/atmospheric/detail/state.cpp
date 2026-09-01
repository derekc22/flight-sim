#include <stdexcept>
#include "simulation/atmospheric/private/detail/state.hpp"
#include "simulation/atmospheric/public/detail/state.hpp"
#include "simulation/frames/public/frame.hpp"
#include "simulation/geography/public/detail/geodesy.hpp"

namespace atmospheric {

    StaticAtmosphericState compute_static_atmospheric_state(const frames::Frame& F, const frames::Frame& E) {
        if (E.id != frames::FrameID::ECEFFrame) {
            throw std::invalid_argument("atmospheric::compute_static_atmospheric_state: E must be ECEFFrame");
        }
        return std_atmosphere(geography::compute_geographic_state(F, E).alt);
    }

}
