#pragma once
#include "simulation/geography/public.hpp"

namespace failures {

    struct FailureInputs {
        geography::HeightAGL height_agl;
    };

    struct FailureManager {

        void check_runtime_failures(const FailureInputs& input);
    };

}