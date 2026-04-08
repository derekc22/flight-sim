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

    Sensor::Sensor(double mean, double stddev, double bias, const Eigen::Vector3d& bias3) : mean(mean), stddev(stddev), bias(bias), bias_3d(bias3), dist(mean, stddev) {}

    double Sensor::_lag(double meas, double prev_meas) {
        return (1-alpha) * meas + alpha * prev_meas;
    }

    Eigen::Vector3d Sensor::_lag(const Eigen::Vector3d& meas, const Eigen::Vector3d& prev_meas) {
        return (1-alpha) * meas + alpha * prev_meas;
    }

    Eigen::Quaterniond Sensor::_lag(const Eigen::Quaterniond& meas, const Eigen::Quaterniond& prev_meas) {
        Eigen::Quaterniond meas_adjusted = meas;
        if (prev_meas.coeffs().dot(meas.coeffs()) < 0.0) meas_adjusted.coeffs() *= -1.0;

        return prev_meas.slerp(1-alpha, meas_adjusted);
    }

    double Sensor::_step(double meas, std::optional<double>& lag_state) {
        double prev_meas = lag_state ? *lag_state : meas;
        double meas_lagged = _lag(meas, prev_meas); // apply EMA
        lag_state = meas_lagged;
        double meas_biased = meas_lagged + bias; // apply bias
        double noise = dist(gen);
        double meas_noised = meas_biased + noise; // apply Gaussian noise
        return meas_noised;
    }

    Eigen::Vector3d Sensor::_step(const Eigen::Vector3d& meas, std::optional<Eigen::Vector3d>& lag_state) {
        Eigen::Vector3d prev_meas = lag_state ? *lag_state : meas;
        Eigen::Vector3d meas_lagged = _lag(meas, prev_meas); // apply EMA
        lag_state = meas_lagged;
        Eigen::Vector3d meas_biased = meas_lagged + bias_3d; // apply bias
        Eigen::Vector3d noise(dist(gen), dist(gen), dist(gen));
        Eigen::Vector3d meas_noised = meas_biased + noise; // apply Gaussian noise
        return meas_noised;
    }

    Eigen::Quaterniond Sensor::_step(const Eigen::Quaterniond& meas, std::optional<Eigen::Quaterniond>& lag_state) {
        Eigen::Quaterniond prev_meas = lag_state ? *lag_state : meas;
        Eigen::Quaterniond meas_lagged = _lag(meas, prev_meas); // apply EMA via quaternion SLERP
        lag_state = meas_lagged;

        double bias_angle = bias_3d.norm();
        Eigen::Quaterniond bias_q;
        if (bias_angle < constants::eps) bias_q = constants::qI;
        else {
            Eigen::Vector3d bias_axis = bias_3d / bias_angle;
            bias_q = Eigen::Quaterniond(Eigen::AngleAxisd(bias_angle, bias_axis));
        }
        Eigen::Quaterniond meas_biased = meas_lagged * bias_q; // apply bias

        Eigen::Vector3d noise_dtheta(dist(gen), dist(gen), dist(gen));
        double angle = noise_dtheta.norm();

        Eigen::Quaterniond dq;
        if (angle < constants::eps) dq = constants::qI;
        else {
            Eigen::Vector3d axis = noise_dtheta / angle;
            dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
        }
        Eigen::Quaterniond meas_noised = meas_biased * dq; // apply Gaussian noise

        meas_noised.normalize();
        return meas_noised;
    }

    AngleOfAttackMeasurement AngleOfAttackVane::_measure(const aerodynamics::AngleOfAttack& alpha) {
        return { _step(alpha.data, prev_alpha_lag) };
    }

    LinearAccelerationMeasurement Accelerometer::_measure(const dynamics::LinearAcceleration& accelB) {
        return { _step(accelB.data, prev_accel_lag) };
    }

    AngularVelocityMeasurement Gyroscope::_measure(const dynamics::AngularVelocity& wB_BI) {
        return { _step(wB_BI.data, prev_wB_BI_lag) };
    }

    StagnationAirPressureMeasurement PitotTube::_measure(const atmospheric::StagnationAirPressure& P0) {
        return { _step(P0.data, prev_P0_lag) };
    }

    StaticAirPressureMeasurement StaticPort::_measure(const atmospheric::StaticAirPressure& P) {
        return { _step(P.data, prev_P_lag) };
    }

    StagnationAirTemperatureMeasurement TotalAirTemperatureProbe::_measure(const atmospheric::StagnationAirTemperature& T0) {
        return { _step(T0.data, prev_T0_lag) };
    }

    PositionMeasurement GNSSReceiver::_measure(const dynamics::Position& pI_BI) {
        return { _step(pI_BI.data, prev_pI_BI_lag) };
    }

    LinearVelocityMeasurement GNSSReceiver::_measure(const dynamics::LinearVelocity& pB_BI_dot) {
        return { _step(pB_BI_dot.data, prev_pB_BI_dot_lag) };
    }
    
    HeadingMeasurement Magnetometer::_measure(const geography::Heading& heading) {
        return { _step(heading.data, prev_heading_lag) };
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
        SensorMeasurements sensor_meas {
            .alpha = sensors.aoa_vane._measure(meas_gt.alpha),
            .accel = sensors.accelerometer._measure(meas_gt.accelB),
            .wB_BI = sensors.gyro._measure(meas_gt.wB_BI),
            .P0 = sensors.pitot_tube._measure(meas_gt.P0),
            .P = sensors.static_port._measure(meas_gt.P),
            .T0 = sensors.tat_probe._measure(meas_gt.T0),
            .pI_BI_gnss = sensors.gnss._measure(meas_gt.pI_BI),
            .pB_BI_dot_gnss = sensors.gnss._measure(meas_gt.pB_BI_dot),
            .heading_BE = sensors.magnetometer._measure(meas_gt.heading)
        };

        MachNumberMeasurement curr_Mach_meas{ atmospheric::compute_mach(sensor_meas.P0, sensor_meas.P).data };
        StaticAirTemperatureMeasurement curr_T_meas{ atmospheric::T_from_T0(sensor_meas.T0, curr_Mach_meas) };
    
        ComputerMeasurements computer_meas {
            .pI_BI_ins = hist.computers ? computers.INS._calculate(hist.computers->pI_BI_ins, hist.computers->pB_BI_dot_ins, sensor_meas.accel, hist.computers->qIB) : PositionMeasurement{ meas_gt.pI_BI.data },
            .pB_BI_dot_ins = hist.computers ? computers.INS._calculate(hist.computers->pB_BI_dot_ins, sensor_meas.accel, hist.computers->pI_BI_ins, hist.computers->qIB, sensor_meas.wB_BI) : LinearVelocityMeasurement{ meas_gt.pB_BI_dot.data },
            .T = curr_T_meas,
            .Mach = curr_Mach_meas,
            .Vinf = computers.ADC._calculate(curr_Mach_meas, curr_T_meas),
            .alt_BE = computers.ADC._calculate(sensor_meas.P),
            .alt_BE_dot = hist.sensors ? computers.ADC._calculate(sensor_meas.P, hist.sensors->P, curr_T_meas) : VerticalSpeedMeasurement{ meas_gt.alt_dot.data },
            .rho = computers.ADC._calculate(sensor_meas.P, curr_T_meas),
            .qIB = hist.computers ? computers.AHRS._calculate(hist.computers->qIB, sensor_meas.wB_BI) : OrientationMeasurement{ meas_gt.qIB.data }
        };

        cache = MeasurementCache{ .sensors = sensor_meas, .computers = computer_meas };
        hist = MeasurementHistory{ .sensors = cache.sensors, .computers = cache.computers };

        return cache;
    }


    std::pair<dynamics::RigidBodyState, aerodynamics::AerodynamicState> get_state_from_avionics(
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
            .p = cache.sensors.pI_BI_gnss,
            .v = cache.sensors.pB_BI_dot_gnss,
            .q = cache.computers.qIB,
            .w = cache.sensors.wB_BI
        };

        aerodynamics::AerodynamicState ads_t_meas = aerodynamics::compute_aerodynamic_state(xN_meas_t, wind);

        // not needed 
        // atmospheric::StaticAtmosphericState static_atmo_meas = {
        //     .P=cache.sensors.P,
        //     .T=cache.computers.T,
        //     .rho=cache.computers.rho,
        //     .mu=static_atmo_t.mu,
        // };

        // geography::GeographicState geo_state_meas = {
        //     .lat = 
        // }
        
        return { xN_meas_t, ads_t_meas };
    }



}
