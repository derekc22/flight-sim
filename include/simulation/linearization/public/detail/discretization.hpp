#pragma once
#include "simulation/linearization/public/data/types.hpp"

namespace linearization {

    DiscretizedLocalLinearization discretize(const LocalLinearization& lin_sol, double dt);

    DiscretizedLocalLinearization discretize_euler(const LocalLinearization& lin_sol, double dt);

}
