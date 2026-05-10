#include "simulation/dynamics/shared.hpp"
#include "simulation/trim/shared.hpp"

namespace trim {

    dynamics::StateVector_T<double> unpack_rigid_body_state(const dynamics::RigidBodyState& xN_t){
        dynamics::TranslationalVelocity vB_BI = xN_t.v;
        dynamics::AngularVelocity wB_BI = xN_t.w;
        dynamics::EulerAngles eulIB;
        eulIB.set(xN_t.q);

        dynamics::State_T<double> xN_t_packed {
            .vx = vB_BI.data(0),
            .vy = vB_BI.data(1),
            .vz = vB_BI.data(2),
            .p = wB_BI.p(),
            .q = wB_BI.q(),
            .r = wB_BI.r(),
            .phi = eulIB.phi(),
            .theta = eulIB.theta(),
        };
        return dynamics::unpack_state_T(xN_t_packed);
    }

}
