#pragma once
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/dimensions.hpp"

namespace allocator {

	using EffectivenessMatrix = constants::MatrixX_T<double, constants::virtual_input_dim, constants::input_dim>;

}
