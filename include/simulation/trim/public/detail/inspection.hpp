#pragma once
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/trim/public/data/types.hpp"

namespace vehicles { struct Aircraft; } // forward declare

namespace trim {

    TrimSolution inspect_trim(vehicles::Aircraft& aircraft, autodiff::AutoDiffModel& model, const atmospheric::Wind& wind);

}
