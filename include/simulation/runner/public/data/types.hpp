#pragma once
#include <string>
#include "analysis/eigenanalysis/public.hpp"
#include "core/devices/public/data/types.hpp"
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/aerodynamics/public/data/types.hpp"
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/control/public/data/types.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/fsm/public/data/types.hpp"
#include "simulation/geography/public/data/types.hpp"
#include "simulation/guidance/public/data/types.hpp"
#include "simulation/linearization/public/data/types.hpp"
#include "simulation/operating/public/data/types.hpp"
#include "simulation/structural/public/data/types.hpp"
#include "simulation/trim/public/data/types.hpp"

namespace vehicles { struct Aircraft; }

namespace runner {

    struct Scheduler;

	struct CLIFlags {
        bool data_flag;
        bool analysis_flag;
        bool fast_flag;
    };

    struct JSONFlags {
        bool trim_flag;
        bool avionics_flag;
        bool control_flag;
        bool estimation_flag;
        bool wind_flag;
        bool verbose_flag;
        bool rerun_flag;
        bool joystick_flag;
    };

    struct ModuleRates {
        double sensor_hz;
        double avionics_hz;
        double estimation_hz;
        double guidance_hz;
        double control_hz;
        double log_hz;
    };

    struct JSONOptions {
        int tf;
        JSONFlags flags;
        ModuleRates module_rates;
    };

    struct CLIOptions {
        std::string aircraft_id;
        CLIFlags flags;
        std::string log_dir_path;
        std::string data_dir_path;
        std::string report_dir_path;
    };

    struct StepContext {
        atmospheric::Wind windI;
        atmospheric::Wind windB;
        dynamics::RigidBodyState Xt;
        dynamics::RigidBodyState XEt;
        structural::StructuralState struc_t;
        aerodynamics::AerodynamicState aero_t;
        geography::GeographicState geo_t;
        atmospheric::StaticAtmosphericState atm_t;
        autodiff::AutoDiffModel autodiff_model;
        operating::OperatingConditions transient_conditions;
        dynamics::RigidBodyState Yt;
        dynamics::RigidBodyState Zt;
        guidance::GuidanceSetpoint setpoint;
        control::ControlOutput u_cmd;
        actuators::ActuatorInputs_T<double> u_actual;
        fsm::FiniteState current_mode = fsm::FiniteState::None;
        dynamics::RigidBodyState Xt1;
        dynamics::Wrench WB_net;
        dynamics::Wrench WB_aerodynamic;
        dynamics::Wrench WB_propulsive;
    };

    struct SchedulerInput {
        fsm::FiniteState current_mode;
        bool fast;
    };

    struct TrimWrapperInput {
        vehicles::Aircraft& aircraft;
        const autodiff::AutoDiffModel& autodiff_model;
        const atmospheric::Wind& windB;
    };

    struct TrimWrapperOutput {
        trim::TrimSolution trim_sol;
    };

    struct LinearizationWrapperInput {
        const autodiff::AutoDiffModel& autodiff_model;
        const trim::TrimSolution& trim_sol;
    };

    struct LinearizationWrapperOutput {
        linearization::LocalLinearization lin_sol;
        linearization::VirtualLocalLinearization virtual_lin_sol;
        analysis::EigenAnalysis eig_sol;
    };

    struct MeasurementsWrapperInput {
        vehicles::Aircraft& aircraft;
        Scheduler& scheduler;
        const StepContext& context;
        const dynamics::Wrench& WB_net_t_1;
        bool enabled;
    };

    struct MeasurementsWrapperOutput {
        dynamics::RigidBodyState Yt;
    };

    struct EstimationWrapperInput {
        vehicles::Aircraft& aircraft;
        Scheduler& scheduler;
        const StepContext& context;
        const trim::TrimSolution& trim_sol;
        const linearization::LocalLinearization& lin_sol;
        const actuators::ActuatorInputs_T<double>& u_actual_t_1;
        bool enabled;
    };

    struct EstimationWrapperOutput {
        dynamics::RigidBodyState Zt;
    };

    struct ControlWrapperInput {
        vehicles::Aircraft& aircraft;
        Scheduler& scheduler;
        const StepContext& context;
        const trim::TrimSolution& trim_sol;
        const linearization::VirtualLocalLinearization& virtual_lin_sol;
        const devices::JoystickManagerOutput& joystick_output;
        fsm::FiniteState current_mode;
    };

    struct ControlWrapperOutput {
        guidance::GuidanceSetpoint setpoint;
        control::ControlOutput u_cmd;
        actuators::ActuatorInputs_T<double> u_actual;
    };

    struct PhysicsWrapperInput {
        vehicles::Aircraft& aircraft;
        const StepContext& context;
    };

    struct PhysicsWrapperOutput {
        dynamics::RigidBodyState Xt1;
        dynamics::Wrench WB_net;
        dynamics::Wrench WB_aerodynamic;
        dynamics::Wrench WB_propulsive;
    };

    struct FlightGearAdapterInput {
        vehicles::Aircraft& aircraft;
        bool wind_enabled;
    };

    struct FlightGearAdapterOutput {
        atmospheric::Wind windI;
        atmospheric::Wind windB;
    };

    struct FlightGearAdapterSendInput {
        const geography::GeographicState& geographic_state;
        const dynamics::EulerAngles& attitude;
    };

    struct RecordingWrapperInput {
        int t;
        const StepContext& context;
        Scheduler& scheduler;
        const JSONFlags& flags;
    };

    struct RunnerInput {
        int t;
    };

    struct RunnerOutput {
        StepContext context;
    };

}
