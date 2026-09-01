#pragma once
#include "simulation/failures/public/data/types.hpp"

namespace failures {

    struct FailureManager {

        void check_runtime_failures(const FailureInputs& input);
    };

}
