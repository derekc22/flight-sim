#include "analysis/eigenanalysis/public.hpp"
#include "simulation/linearization/public/detail/jacobian.hpp"
#include "simulation/runner/public/components/linearization.hpp"

namespace runner {

    LinearizationOutput Linearization::step(const LinearizationInput& input) {
        // compute linearization
        physical = linearization::linearize_operating_point(
            input.autodiff_model,
            input.trim_solution.operating_point,
            input.trim_solution.conditions
        );

        // compute virtual linearization
        virtual_model = linearization::linearize_virtual_operating_point(
            input.autodiff_model,
            operating::VirtualOperatingPoint_T<double>{
                .state=input.trim_solution.operating_point.state,
                .input=input.trim_solution.wrench
            }
        );

        // perform eigenanalysis
        eigenanalysis = analysis::linearization_eigen_analysis(physical);

        return {
            .physical = physical,
            .virtual_model = virtual_model,
            .eigenanalysis = eigenanalysis
        };
    }

}
