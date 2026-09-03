#include "analysis/eigenanalysis/public.hpp"
#include "simulation/linearization/public/detail/jacobian.hpp"
#include "simulation/runner/public/wrappers/linearization.hpp"

namespace runner {

    LinearizationWrapperOutput LinearizationWrapper::step(const LinearizationWrapperInput& input) {
        // compute linearization
        lin_sol = linearization::linearize_operating_point(
            input.autodiff_model,
            input.trim_sol.operating_point,
            input.trim_sol.conditions
        );

        // compute virtual linearization
        virtual_lin_sol = linearization::linearize_virtual_operating_point(
            input.autodiff_model,
            operating::VirtualOperatingPoint_T<double>{
                .state=input.trim_sol.operating_point.state,
                .input=input.trim_sol.wrench
            }
        );

        // perform eigenanalysis
        eig_sol = analysis::linearization_eigen_analysis(lin_sol);

        return {
            .lin_sol = lin_sol,
            .virtual_lin_sol = virtual_lin_sol,
            .eig_sol = eig_sol
        };
    }

}
