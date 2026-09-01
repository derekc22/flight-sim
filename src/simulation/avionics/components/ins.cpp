#include "simulation/avionics/public/components/ins.hpp"
#include "simulation/integrators/public/detail/euler.hpp"

namespace avionics {

    sensors::PositionMeasurement InertialNavigationSystem::compute(const sensors::PositionMeasurement& prev_pI_BI, const sensors::TranslationalVelocityMeasurement& prev_vB_BI, const sensors::TranslationalAccelerationMeasurement& fB, const dynamics::Gravity& gB, const OrientationMeasurement& prev_qIB, double dt) {
        dynamics::TranslationalVelocity pI_BI_dot{ prev_qIB.data.conjugate() * prev_vB_BI.data };
        dynamics::TranslationalAcceleration aI_BI{ prev_qIB.data.conjugate() * (fB.data + gB.data) };
        return { integrators::trans_kin(prev_pI_BI, pI_BI_dot, aI_BI, dt).data };
    }

    sensors::TranslationalVelocityMeasurement InertialNavigationSystem::compute(const sensors::TranslationalVelocityMeasurement& prev_vB_BI, const sensors::TranslationalAccelerationMeasurement& fB, const dynamics::Gravity& gB, const sensors::AngularVelocityMeasurement& wB_BI, double dt) {
        dynamics::TranslationalAcceleration vB_BI_dot{ fB.data + gB.data - wB_BI.data.cross(prev_vB_BI.data) };
        return { integrators::trans_kin_vel(prev_vB_BI, vB_BI_dot, dt).data };
    }

}
