#include <Eigen/Dense>
#include "simulation/integrators/private/detail/wrench.hpp"
#include "simulation/integrators/public/detail/wrench.hpp"
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/geography/public/detail/gravity.hpp"
#include "simulation/propulsion/public/manager.hpp"

namespace integrators {

    WrenchEvaluation compute_net_wrench(const dynamics::RigidBodyState& Xt, RK4Model& model, const operating::OperatingConditions& conditions, const actuators::ActuatorInputs_T<double>& u, double dt) {
        const dynamics::Twist_T<double> twist{
            .v = Xt.v.data,
            .w = Xt.w.data
        };

        const atmospheric::Wind windB{ Xt.q.data * conditions.windI.data };
        const Eigen::Vector3d gB = geography::gB(Xt.q).data;

        const WrenchEvaluation_T<double> evaluation = compute_wrench_set_T<double>(
            model, 
            twist, 
            conditions.atm, 
            u, 
            windB, 
            gB,
            dt,
            conditions.steady_state
        );

        const dynamics::WrenchSet_T<double>& wrench = evaluation.wrench;

        return {
            .wrench = {
                .aerodynamic = {
                    .F = dynamics::Force{ wrench.aerodynamic.F },
                    .M = dynamics::Moment{ wrench.aerodynamic.M }
                },
                .propulsive = {
                    .F = dynamics::Force{ wrench.propulsive.F },
                    .M = dynamics::Moment{ wrench.propulsive.M }
                },
                .net = {
                    .F = dynamics::Force{ wrench.net.F },
                    .M = dynamics::Moment{ wrench.net.M }
                }
            },
            .next_propulsion_state = evaluation.next_propulsion_state
        };
    }

}
