#pragma once
#include <vector>
#include "simulation/aerodynamics/public/data/types.hpp"

namespace aerodynamics {

    struct AerodynamicsManager {
        std::vector<Surface> surfaces;

        AerodynamicsManager(std::vector<Surface> s);
        void compute_surface_geometry();
        AerodynamicsManagerOutput step(const AerodynamicsManagerInput& input);
    };

}
