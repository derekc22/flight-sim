#include <random>
#include <utility> // For std::pair
#include "simulation/avionics/avionics.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/transforms/transforms.hpp"
#include "simulation/geography/geography.hpp"

namespace avionics {

    void MeasurementCache::update_from_sensor_cache(const MeasurementCache& sensor_cache) {
        curr_alpha_meas = sensor_cache.curr_alpha_meas;
        curr_accel_meas = sensor_cache.curr_accel_meas;
        curr_wB_BI_meas = sensor_cache.curr_wB_BI_meas;
        curr_P0_meas = sensor_cache.curr_P0_meas;
        curr_P_meas = sensor_cache.curr_P_meas;
        curr_T0_meas = sensor_cache.curr_T0_meas;
        curr_pI_BI_gnss_meas = sensor_cache.curr_pI_BI_gnss_meas;
        curr_pB_BI_dot_gnss_meas = sensor_cache.curr_pB_BI_dot_gnss_meas;
        curr_heading_BE_meas = sensor_cache.curr_heading_BE_meas;
    }

    void MeasurementCache::update_from_computer_cache(const MeasurementCache& computer_cache) {
        curr_pI_BI_ins_meas = computer_cache.curr_pI_BI_ins_meas;
        curr_pB_BI_dot_ins_meas = computer_cache.curr_pB_BI_dot_ins_meas;
        curr_T_meas = computer_cache.curr_T_meas;
        curr_Mach_meas = computer_cache.curr_Mach_meas;
        curr_qIB_meas = computer_cache.curr_qIB_meas;
        curr_Vinf_meas = computer_cache.curr_Vinf_meas;
        curr_alt_BE_meas = computer_cache.curr_alt_BE_meas;
        curr_alt_BE_dot_meas = computer_cache.curr_alt_BE_dot_meas;
        curr_rho_meas = computer_cache.curr_rho_meas;
    }

    MeasurementHistory AvionicsProperties::hist_from_cache(const MeasurementCache& cache) {
        return {
            .prev_alpha_meas = cache.curr_alpha_meas,
            .prev_accel_meas = cache.curr_accel_meas,
            .prev_wB_BI_meas = cache.curr_wB_BI_meas,
            .prev_P0_meas = cache.curr_P0_meas,
            .prev_P_meas = cache.curr_P_meas,
            .prev_T0_meas = cache.curr_T0_meas,
            .prev_pI_BI_gnss_meas = cache.curr_pI_BI_gnss_meas,
            .prev_pB_BI_dot_gnss_meas = cache.curr_pB_BI_dot_gnss_meas,
            .prev_heading_BE_meas = cache.curr_heading_BE_meas,
            .prev_pI_BI_ins_meas = cache.curr_pI_BI_ins_meas,
            .prev_pB_BI_dot_ins_meas = cache.curr_pB_BI_dot_ins_meas,
            .prev_T_meas = cache.curr_T_meas,
            .prev_Mach_meas = cache.curr_Mach_meas,
            .prev_qIB_meas = cache.curr_qIB_meas,
            .prev_Vinf_meas = cache.curr_Vinf_meas,
            .prev_alt_BE_meas = cache.curr_alt_BE_meas,
            .prev_alt_dot_meas = cache.curr_alt_BE_dot_meas,
            .prev_rho_meas = cache.curr_rho_meas
        };
    }

    Sensor::Sensor(double mean, double stddev, double bias, const Eigen::Vector3d& bias3) : mean(mean), stddev(stddev), bias(bias), bias_3d(bias3), dist(mean, stddev) {}

    double Sensor::_step(double meas, double prev_meas){
        double meas_lagged = (1-alpha) * meas + alpha * prev_meas; // apply EMA
        double meas_biased = meas_lagged + bias; // apply bias
        double noise = dist(gen);
        double meas_noised = meas_biased + noise; // apply Gaussian noise
        return meas_noised;
    }

    Eigen::Vector3d Sensor::_step(const Eigen::Vector3d& meas, const Eigen::Vector3d& prev_meas){
        Eigen::Vector3d meas_lagged = (1-alpha) * meas + alpha * prev_meas; // apply EMA
        Eigen::Vector3d meas_biased = meas_lagged + bias_3d; // apply bias
        Eigen::Vector3d noise(dist(gen), dist(gen), dist(gen));
        Eigen::Vector3d meas_noised = meas_biased + noise; // apply Gaussian noise
        return meas_noised;
    }

    Eigen::Quaterniond Sensor::_step(const Eigen::Quaterniond& meas, const Eigen::Quaterniond& prev_meas){
        Eigen::Quaterniond meas_adjusted = meas;
        if (prev_meas.coeffs().dot(meas.coeffs()) < 0.0) meas_adjusted.coeffs() *= -1.0;

        Eigen::Quaterniond meas_lagged = prev_meas.slerp(1-alpha, meas_adjusted); // apply EMA via quaternion SLERP

        double bias_angle = bias_3d.norm();
        Eigen::Quaterniond bias_q;
        if (bias_angle < constants::eps) bias_q = Eigen::Quaterniond::Identity();
        else {
            Eigen::Vector3d bias_axis = bias_3d / bias_angle;
            bias_q = Eigen::Quaterniond(Eigen::AngleAxisd(bias_angle, bias_axis));
        }
        Eigen::Quaterniond meas_biased = meas_lagged * bias_q; // apply bias

        Eigen::Vector3d noise_dtheta(dist(gen), dist(gen), dist(gen));
        double angle = noise_dtheta.norm();

        Eigen::Quaterniond dq;
        if (angle < constants::eps) dq = Eigen::Quaterniond::Identity();
        else {
            Eigen::Vector3d axis = noise_dtheta / angle;
            dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
        }
        Eigen::Quaterniond meas_noised = meas_biased * dq; // apply Gaussian noise

        meas_noised.normalize();
        return meas_noised;
    }

    AngleOfAttackMeasurement AngleOfAttackVane::_measure(const aerodynamics::AngleOfAttack& alpha, const aerodynamics::AngleOfAttack& prev_alpha) {
        return { _step(alpha.data, prev_alpha.data) };
    }

    LinearAccelerationMeasurement Accelerometer::_measure(const dynamics::LinearAcceleration& accelB, const dynamics::LinearAcceleration& prev_accel) {
        return { _step(accelB.data, prev_accel.data) };
    }

    AngularVelocityMeasurement Gyroscope::_measure(const dynamics::AngularVelocity& wB_BI, const dynamics::AngularVelocity& prev_wB_BI) {    
        return { _step(wB_BI.data, prev_wB_BI.data) };
    }

    StagnationAirPressureMeasurement PitotTube::_measure(const atmospheric::StagnationAirPressure& P0, const atmospheric::StagnationAirPressure& prev_P0) {
        return { _step(P0.data, prev_P0.data) };
    }

    StaticAirPressureMeasurement StaticPort::_measure(const atmospheric::StaticAirPressure& P, const atmospheric::StaticAirPressure& prev_P) {
        return { _step(P.data, prev_P.data) };
    }

    StagnationAirTemperatureMeasurement TotalAirTemperatureProbe::_measure(const atmospheric::StagnationAirTemperature& T0, const atmospheric::StagnationAirTemperature& prev_T0) {
        return { _step(T0.data, prev_T0.data) };
    }

    PositionMeasurement GNSSReceiver::_measure(const dynamics::Position& pI_BI, const dynamics::Position& prev_pI_BI) {
        return { _step(pI_BI.data, prev_pI_BI.data) };
    }

    LinearVelocityMeasurement GNSSReceiver::_measure(const dynamics::LinearVelocity& pB_BI_dot, const dynamics::LinearVelocity& prev_pB_BI_dot) {
        return { _step(pB_BI_dot.data, prev_pB_BI_dot.data) };
    }
    
    HeadingMeasurement Magnetometer::_measure(const geography::Heading& heading, const geography::Heading& prev_heading) {
        return { _step(heading.data, prev_heading.data) };
    }

    FreeStreamVelocityMeasurement AirDataComputer::_calculate(const MachNumberMeasurement& Mach, const StaticAirTemperatureMeasurement& T) {
        double Vinf = Mach.data * std::sqrt(constants::gamma_air * constants::R_air * T.data);
        return { Vinf };
    }

    AltitudeMeasurement AirDataComputer::_calculate(const StaticAirPressureMeasurement& P) {
        double alt = (constants::T_SL / std::abs(constants::lapse_rate)) * (1.0 - std::pow(P.data / constants::P_SL, (constants::R_air * std::abs(constants::lapse_rate)) / constants::g_earth));
        return { alt };
    }

    VerticalSpeedMeasurement AirDataComputer::_calculate(const StaticAirPressureMeasurement& P, const StaticAirPressureMeasurement& prev_P, const StaticAirTemperatureMeasurement& T) {
        double P_dot_meas = (P.data - prev_P.data) / constants::dt;
        double alt_dot = -(constants::R_air * T.data / (constants::g_earth * P.data)) * P_dot_meas;
        return { alt_dot };
    }

    AirDensityMeasurement AirDataComputer::_calculate(const StaticAirPressureMeasurement& P, const StaticAirTemperatureMeasurement& T) {
        double rho = P.data / (constants::R_air * T.data);
        return { rho };
    }

    OrientationMeasurement AttitudeHeadingReferenceSystem::_calculate(const OrientationMeasurement& prev_qIB, const AngularVelocityMeasurement& wB_BI) {
        return { dynamics::_quat_kin(prev_qIB, wB_BI) };
    }

    PositionMeasurement InertialNavigationSystem::_calculate(const PositionMeasurement& prev_pI_BI, const LinearVelocityMeasurement& prev_pB_BI_dot, const LinearAccelerationMeasurement& accelB, const OrientationMeasurement& prev_qIB) {
        dynamics::Gravity prev_gB = geography::gB(prev_pI_BI, prev_qIB);
        dynamics::LinearVelocity pI_BI_dot{ prev_qIB.data.conjugate() * prev_pB_BI_dot.data };
        dynamics::LinearAcceleration pI_BI_ddot{ prev_qIB.data.conjugate() * (accelB.data + prev_gB.data) };
        return { dynamics::_trans_kin(prev_pI_BI, pI_BI_dot, pI_BI_ddot).data };
    }

    LinearVelocityMeasurement InertialNavigationSystem::_calculate(const LinearVelocityMeasurement& prev_pB_BI_dot, const LinearAccelerationMeasurement& accelB, const PositionMeasurement& prev_pI_BI, const OrientationMeasurement& prev_qIB, const AngularVelocityMeasurement& wB_BI) {
        dynamics::Gravity prev_gB = geography::gB(prev_pI_BI, prev_qIB);
        dynamics::LinearAcceleration pB_BI_ddot{ accelB.data + prev_gB.data - wB_BI.data.cross(prev_pB_BI_dot.data) };
        return { dynamics::_trans_kin_vel(prev_pB_BI_dot, pB_BI_ddot).data };
    }


    MeasurementCache AvionicsProperties::step(const MeasurementGroundTruth& meas_gt) {
        aerodynamics::AngleOfAttack prev_alpha = hist.prev_alpha_meas ? aerodynamics::AngleOfAttack{ hist.prev_alpha_meas->data } : meas_gt.alpha;
        dynamics::LinearAcceleration prev_accel = hist.prev_accel_meas ? dynamics::LinearAcceleration{ hist.prev_accel_meas->data } : meas_gt.accelB;
        dynamics::AngularVelocity prev_wB_BI = hist.prev_wB_BI_meas ? dynamics::AngularVelocity{ hist.prev_wB_BI_meas->data } : meas_gt.wB_BI;
        atmospheric::StagnationAirPressure prev_P0 = hist.prev_P0_meas ? atmospheric::StagnationAirPressure{ hist.prev_P0_meas->data } : meas_gt.P0;
        atmospheric::StaticAirPressure prev_P = hist.prev_P_meas ? atmospheric::StaticAirPressure{ hist.prev_P_meas->data } : meas_gt.P;
        atmospheric::StagnationAirTemperature prev_T0 = hist.prev_T0_meas ? atmospheric::StagnationAirTemperature{ hist.prev_T0_meas->data } : meas_gt.T0;
        dynamics::Position prev_pI_BI_gnss = hist.prev_pI_BI_gnss_meas ? dynamics::Position{ hist.prev_pI_BI_gnss_meas->data } : meas_gt.pI_BI;
        dynamics::LinearVelocity prev_pB_BI_dot_gnss = hist.prev_pB_BI_dot_gnss_meas ? dynamics::LinearVelocity{ hist.prev_pB_BI_dot_gnss_meas->data } : meas_gt.pB_BI_dot;
        geography::Heading prev_heading = hist.prev_heading_BE_meas ? geography::Heading{ hist.prev_heading_BE_meas->data } : meas_gt.heading;

        MeasurementCache sensor_cache {
            .curr_alpha_meas = sensors.aoa_vane._measure(meas_gt.alpha, prev_alpha),
            .curr_accel_meas = sensors.accelerometer._measure(meas_gt.accelB, prev_accel),
            .curr_wB_BI_meas = sensors.gyro._measure(meas_gt.wB_BI, prev_wB_BI),
            .curr_P0_meas = sensors.pitot_tube._measure(meas_gt.P0, prev_P0),
            .curr_P_meas = sensors.static_port._measure(meas_gt.P, prev_P),
            .curr_T0_meas = sensors.tat_probe._measure(meas_gt.T0, prev_T0),
            .curr_pI_BI_gnss_meas = sensors.gnss._measure(meas_gt.pI_BI, prev_pI_BI_gnss),
            .curr_pB_BI_dot_gnss_meas = sensors.gnss._measure(meas_gt.pB_BI_dot, prev_pB_BI_dot_gnss),
            .curr_heading_BE_meas = sensors.magnetometer._measure(meas_gt.heading, prev_heading)
        };

        cache.update_from_sensor_cache(sensor_cache);


        MachNumberMeasurement curr_Mach_meas{ atmospheric::compute_mach(cache.curr_P0_meas, cache.curr_P_meas).data };
        StaticAirTemperatureMeasurement curr_T_meas{ atmospheric::T_from_T0(cache.curr_T0_meas, curr_Mach_meas) };
    
        MeasurementCache computer_cache {
            .curr_pI_BI_ins_meas = hist.prev_pI_BI_ins_meas && hist.prev_pB_BI_dot_ins_meas && hist.prev_qIB_meas ? computers.INS._calculate(hist.prev_pI_BI_ins_meas.value(), hist.prev_pB_BI_dot_ins_meas.value(), cache.curr_accel_meas, hist.prev_qIB_meas.value()) : PositionMeasurement{ meas_gt.pI_BI.data },
            .curr_pB_BI_dot_ins_meas = hist.prev_pB_BI_dot_ins_meas && hist.prev_pI_BI_ins_meas && hist.prev_qIB_meas ? computers.INS._calculate(hist.prev_pB_BI_dot_ins_meas.value(), cache.curr_accel_meas, hist.prev_pI_BI_ins_meas.value(), hist.prev_qIB_meas.value(), cache.curr_wB_BI_meas) : LinearVelocityMeasurement{ meas_gt.pB_BI_dot.data },
            .curr_T_meas = curr_T_meas,
            .curr_Mach_meas = curr_Mach_meas,
            .curr_Vinf_meas = computers.ADC._calculate(curr_Mach_meas, curr_T_meas),
            .curr_alt_BE_meas = computers.ADC._calculate(cache.curr_P_meas),
            .curr_alt_BE_dot_meas = hist.prev_P_meas ? computers.ADC._calculate(cache.curr_P_meas, hist.prev_P_meas.value(), curr_T_meas) : VerticalSpeedMeasurement{ meas_gt.alt_dot.data },
            .curr_rho_meas = computers.ADC._calculate(cache.curr_P_meas, curr_T_meas),
            .curr_qIB_meas = hist.prev_qIB_meas ? computers.AHRS._calculate(hist.prev_qIB_meas.value(), cache.curr_wB_BI_meas) : OrientationMeasurement{ meas_gt.qIB.data }
        };

        cache.update_from_computer_cache(computer_cache);
        hist = hist_from_cache(cache);

        return cache;
    }


    std::pair<dynamics::RigidBodyState, aerodynamics::AerodynamicState> update_state_from_avionics(
        const dynamics::RigidBodyState& xN_t, 
        const aerodynamics::AerodynamicState& ads_t, 
        const atmospheric::StaticAtmosphericState& static_atmo_t, 
        const geography::GeographicState& geo_t,
        const dynamics::Mass& mass,
        const atmospheric::Wind & wind,
        const dynamics::Wrench WB_net,
        AvionicsProperties& avionics_properties
    ) {

        atmospheric::MachNumber Mach = atmospheric::ms_to_mach(xN_t.v, static_atmo_t.T);
        atmospheric::StagnationAtmosphericState stagnation_atmo_t = atmospheric::static_to_stagnation(static_atmo_t, Mach);
        dynamics::LinearVelocity vI_BI{ xN_t.q.data.conjugate() * xN_t.v.data };
        double alt_dot = xN_t.p.data.normalized().dot(vI_BI.data);

        dynamics::EulerAngles eul;
        eul.set(xN_t.q);

        avionics::MeasurementGroundTruth meas_gt = {
            .alpha = ads_t.alpha,
            .accelB = dynamics::LinearAcceleration{ WB_net.F.data / mass.data - geography::gB(xN_t.p, xN_t.q).data },
            .wB_BI = xN_t.w,
            .P0 = stagnation_atmo_t.P0,
            .P = static_atmo_t.P,
            .T0 = stagnation_atmo_t.T0,
            .pI_BI = xN_t.p,
            .pB_BI_dot = xN_t.v,

            .T = static_atmo_t.T,
            .Mach = Mach,
            // .heading = geography::Heading{ transforms::_quatC_to_eul_intr(xN_t.q.data, "ZYX")(0) },
            .heading = geography::Heading{ eul.psi() },
            .qIB = xN_t.q,
            .Vinf = ads_t.Vinf,
            .alt_BE = geo_t.alt,
            .alt_dot = dynamics::VerticalSpeed{ alt_dot },
            .rho = static_atmo_t.rho,
        };
        
        MeasurementCache cache = avionics_properties.step(meas_gt);

        dynamics::RigidBodyState xN_meas_t = { 
            .p = cache.curr_pI_BI_gnss_meas,
            .v = cache.curr_pB_BI_dot_gnss_meas,
            .q = cache.curr_qIB_meas,
            .w = cache.curr_wB_BI_meas
        };

        aerodynamics::AerodynamicState ads_t_meas = aerodynamics::compute_aerodynamic_state(xN_meas_t, wind);

        // not needed 
        // atmospheric::StaticAtmosphericState static_atmo_meas = {
        //     .P=cache.curr_P_meas,
        //     .T=cache.curr_T_meas,
        //     .rho=cache.curr_rho_meas,
        //     .mu=static_atmo_t.mu,
        // };

        // geography::GeographicState geo_state_meas = {
        //     .lat = 
        // }
        
        return { xN_meas_t, ads_t_meas };
    }



}
