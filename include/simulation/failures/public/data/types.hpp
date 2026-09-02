#pragma once
#include "simulation/geography/public/data/types.hpp"

namespace failures {

    struct FailureManagerInput {
        geography::HeightAGL height_agl;
    };

    struct FailureManagerOutput {
        bool terrain_collision;
    };

}
