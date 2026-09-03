#include <stdexcept>
#include <spdlog/spdlog.h>
#include "simulation/aerodynamics/public/detail/state.hpp"
#include "simulation/atmospheric/public/detail/state.hpp"
#include "simulation/trim/private/data/types.hpp"
#include "simulation/trim/private/detail/solver.hpp"
#include "simulation/trim/public/data/helpers.hpp"
#include "simulation/trim/public/detail/inspection.hpp"
#include "simulation/vehicles/public/aircraft.hpp"

namespace trim {

    TrimSolution inspect_trim(vehicles::Aircraft& aircraft, const autodiff::AutoDiffModel& model, const atmospheric::Wind& wind) {

        const actuators::PropulsorActuatorInputs_T<double>& propulsor_limit_max = model.actuator_limits.limit_max.propulsor_inputs;
        const actuators::PropulsorActuatorInputs_T<double>& propulsor_limit_min = model.actuator_limits.limit_min.propulsor_inputs;

        const aerodynamics::AerodynamicState target_aero = aerodynamics::compute_aerodynamic_state(aircraft.CGFrameFRD, aircraft.NEDFrameECEF, wind);

        const TrimProblem problem{
            .target = TrimTarget{
                .beta = target_aero.beta.data,
                .phi = aircraft.FRDFrameNED.eulNB.phi(),
                .theta = aircraft.FRDFrameNED.eulNB.theta(),
                .vx = aircraft.FRDFrameNED.vB_BN.data(0),
                .vz = aircraft.FRDFrameNED.vB_BN.data(2),
                .psi_dot = aircraft.FRDFrameNED.eulNB_dot.psi_dot()
            },
            .conditions = operating::OperatingConditions{
                .atm = atmospheric::compute_static_atmospheric_state(aircraft.FRDFrameECEF, aircraft.ECEFFrame),
                .windB = wind,
                .steady_state = true
            },
            .initial_guess = operating::OperatingPoint_T<double>{
                .state = dynamics::State_T<double>{
                    .vx = aircraft.FRDFrameNED.vB_BN.data.x(),
                    .vy = aircraft.FRDFrameNED.vB_BN.data.y(),
                    .vz = aircraft.FRDFrameNED.vB_BN.data.z(),
                    .p = aircraft.FRDFrameNED.wB_BN.p(),
                    .q = aircraft.FRDFrameNED.wB_BN.q(),
                    .r = aircraft.FRDFrameNED.wB_BN.r(),
                    .phi = aircraft.FRDFrameNED.eulNB.phi(),
                    .theta = aircraft.FRDFrameNED.eulNB.theta(),
                },
                .input = actuators::ActuatorInputs_T<double>{
                    .surface_inputs = {
                        .elevator_cmd = 0,
                        .aileron_cmd = 0,
                        .rudder_cmd = 0,
                    },
                    .propulsor_inputs = {
                        .front_propulsor_cmd = 0.5 * (
                            propulsor_limit_max.front_propulsor_cmd +
                            propulsor_limit_min.front_propulsor_cmd
                        ),
                        .left_propulsor_cmd = 0.5 * (
                            propulsor_limit_max.left_propulsor_cmd +
                            propulsor_limit_min.left_propulsor_cmd
                        ),
                        .right_propulsor_cmd = 0.5 * (
                            propulsor_limit_max.right_propulsor_cmd +
                            propulsor_limit_min.right_propulsor_cmd
                        )
                    }
                }
            }
        };

        const TrimSolution trim_sol = solve_trim(problem, model);

        if (!trim_sol.converged) {
            spdlog::error(print_trim_solution(trim_sol));
            throw std::runtime_error("trim::inspect_trim: Error, trim failed to converge");
        }

        return trim_sol;
    }

}
