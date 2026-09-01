#include <utility>
#include "simulation/aerodynamics/public/detail/loads.hpp"
#include "simulation/aerodynamics/public/manager.hpp"
#include "simulation/aerodynamics/public/detail/state.hpp"

namespace aerodynamics {

    AerodynamicsManager::AerodynamicsManager(std::vector<Surface> s) : surfaces(std::move(s)) {
        compute_surface_geometry();
    }

    void AerodynamicsManager::compute_surface_geometry() {
        for (Surface& s : surfaces) {
            s.area = s.chord * s.span;
            s.AR   = s.span / s.chord;
        }
    }

    AerodynamicsManagerOutput AerodynamicsManager::step(const AerodynamicsManagerInput& input) {
        return {
            .aerodynamic_state = compute_aerodynamic_state(input.state, input.wind),
            .aerodynamic_wrench = step_aero_forces_moments(surfaces, input.center_of_gravity, input.state, input.atmosphere, input.actuator_inputs, input.wind)
        };
    }

}
