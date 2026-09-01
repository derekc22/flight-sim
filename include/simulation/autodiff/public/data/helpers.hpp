#pragma once
#include "simulation/autodiff/public/data/types.hpp"

namespace vehicles { struct Aircraft; } // forward declare

namespace autodiff {

    AutoDiffModel build_autodiff_model(vehicles::Aircraft& aircraft, const structural::StructuralState& struc_t);

}
