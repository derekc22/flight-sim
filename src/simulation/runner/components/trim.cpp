#include "simulation/runner/public/components/trim.hpp"
#include "simulation/trim/public/detail/inspection.hpp"

namespace runner {

    TrimOutput Trim::step(const TrimInput& input) {
        solution = trim::inspect_trim(input.aircraft, input.autodiff_model, input.wind);
        return { .solution = solution };
    }

}
