#pragma once
#include <string_view>
#include "simulation/fsm/public.hpp"


namespace fsm {

	std::string_view mode_to_string(FiniteState mode);

}