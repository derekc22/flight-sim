#include "simulation/avionics/public/components/ahrs.hpp"
#include "simulation/geography/public/detail/gravity.hpp"
#include "simulation/integrators/public/detail/euler.hpp"
#include "simulation/util/public.hpp"

namespace avionics {

    OrientationMeasurement AttitudeHeadingReferenceSystem::compute(const OrientationMeasurement& prev_qIB, const sensors::AngularVelocityMeasurement& wB_BI, const sensors::TranslationalAccelerationMeasurement& fB, double dt) {
        // Mahony filter
        dynamics::OrientationQuaternion qIB_pred = integrators::quat_kin(prev_qIB, wB_BI, dt);
        if (util::abs(fB.data.norm() - constants::g_earth) > fB_tol) {
            return { qIB_pred };
        }

        Eigen::Vector3d gB_pred = qIB_pred.data * geography::gN().data;
        Eigen::Vector3d gB_meas = -fB.data;
        Eigen::Vector3d err = -gB_pred.normalized().cross(gB_meas.normalized());

        integral += err * dt;
        dynamics::AngularVelocity wB_BI_corrected{ wB_BI.data + Kp * err + Ki * integral };

        return { integrators::quat_kin(prev_qIB, wB_BI_corrected, dt) };

    }

}
