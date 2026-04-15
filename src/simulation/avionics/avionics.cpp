#include "simulation/avionics/avionics.hpp"

namespace avionics {

    MeasurementCache AvionicsProperties::step(const MeasurementGroundTruth& meas_gt) {
        SensorMeasurements sensor_meas {
            .alpha = sensors.aoa_vane._measure(meas_gt.alpha),
            .accel = sensors.accelerometer._measure(meas_gt.accelB),
            .wB_BI = sensors.gyro._measure(meas_gt.wB_BI),
            .P0 = sensors.pitot_tube._measure(meas_gt.P0),
            .P = sensors.static_port._measure(meas_gt.P),
            .T0 = sensors.tat_probe._measure(meas_gt.T0),
            .pI_BI_gnss = sensors.gnss._measure(meas_gt.pI_BI),
            .vB_BI_gnss = sensors.gnss._measure(meas_gt.vB_BI),
            .heading_BE = sensors.magnetometer._measure(meas_gt.heading)
        };

        MachNumberMeasurement curr_Mach_meas{ atmospheric::compute_mach(sensor_meas.P0, sensor_meas.P).data };
        StaticAirTemperatureMeasurement curr_T_meas{ atmospheric::T_from_T0(sensor_meas.T0, curr_Mach_meas) };
    
        ComputerMeasurements computer_meas {
            .pI_BI_ins = hist.computers ? computers.INS._calculate(hist.computers->pI_BI_ins, hist.computers->vB_BI_ins, sensor_meas.accel, hist.computers->qIB) : PositionMeasurement{ meas_gt.pI_BI.data },
            .vB_BI_ins = hist.computers ? computers.INS._calculate(hist.computers->vB_BI_ins, sensor_meas.accel, hist.computers->pI_BI_ins, hist.computers->qIB, sensor_meas.wB_BI) : LinearVelocityMeasurement{ meas_gt.vB_BI.data },
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


    // std::pair<dynamics::RigidBodyState, aerodynamics::AerodynamicState> get_state_from_avionics(
    dynamics::RigidBodyState get_state_from_avionics(
        const dynamics::RigidBodyState& xN_t, 
        const aerodynamics::AerodynamicState& ads_t, 
        const atmospheric::StaticAtmosphericState& static_atmo_t, 
        const geography::GeographicState& geo_t,
        const dynamics::Mass& mass,
        const atmospheric::Wind & wind,
        const dynamics::Wrench& WB_net,
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
            .vB_BI = xN_t.v,

            .T = static_atmo_t.T,
            .Mach = Mach,
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
            .v = cache.sensors.vB_BI_gnss,
            .q = cache.computers.qIB,
            .w = cache.sensors.wB_BI
        };

        // aerodynamics::AerodynamicState ads_t_meas = aerodynamics::compute_aerodynamic_state(xN_meas_t, wind);

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
        
        // return { xN_meas_t, ads_t_meas };
        return xN_meas_t;
    }

}
