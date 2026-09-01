#pragma once
#include <string>
#include "analysis/eigenanalysis/public.hpp"
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

    struct TrimInput {
        vehicles::Aircraft& aircraft;
        autodiff::AutoDiffModel& autodiff_model;
        const atmospheric::Wind& wind;
    };

    struct TrimOutput {
        trim::TrimSolution solution;
    };

    struct LinearizationInput {
        autodiff::AutoDiffModel& autodiff_model;
        const trim::TrimSolution& trim_solution;
    };

    struct LinearizationOutput {
        linearization::LocalLinearization physical;
        linearization::VirtualLocalLinearization virtual_model;
        analysis::EigenAnalysis eigenanalysis;
    };

    struct MeasurementsInput {
        vehicles::Aircraft& aircraft;
        Scheduler& scheduler;
        const StepContext& context;
        const dynamics::Wrench& previous_wrench;
        bool enabled;
    };

    struct MeasurementsOutput {
        dynamics::RigidBodyState measured_state;
    };

    struct EstimationInput {
        vehicles::Aircraft& aircraft;
        Scheduler& scheduler;
        const StepContext& context;
        const trim::TrimSolution& trim_solution;
        const linearization::LocalLinearization& linearization;
        const actuators::ActuatorInputs_T<double>& actual_inputs;
        bool enabled;
    };

    struct EstimationOutput {
        dynamics::RigidBodyState estimated_state;
    };

    struct ControlInput {
        vehicles::Aircraft& aircraft;
        Scheduler& scheduler;
        const StepContext& context;
        const trim::TrimSolution& trim_solution;
        const linearization::VirtualLocalLinearization& linearization;
    };

    struct ControlOutput {
        guidance::GuidanceSetpoint setpoint;
        control::ControlOutput commanded_inputs;
        actuators::ActuatorInputs_T<double> actual_inputs;
        fsm::FiniteState current_mode;
    };

    struct PhysicsInput {
        vehicles::Aircraft& aircraft;
        const StepContext& context;
    };

    struct PhysicsOutput {
        dynamics::RigidBodyState next_state;
        dynamics::Wrench net_wrench;
        dynamics::Wrench aerodynamic_wrench;
        dynamics::Wrench propulsive_wrench;
    };

    struct FlightGearInput {
        vehicles::Aircraft& aircraft;
        bool wind_enabled;
    };

    struct FlightGearOutput {
        atmospheric::Wind inertial_wind;
        atmospheric::Wind body_wind;
    };

    struct FlightGearSendInput {
        const geography::GeographicState& geographic_state;
        const dynamics::EulerAngles& attitude;
    };

    struct RecordingInput {
        int t;
        const StepContext& context;
        Scheduler& scheduler;
        const JSONFlags& flags;
    };

    struct RunManagerInput {
        int t;
    };

    struct RunManagerOutput {
        StepContext context;
    };

}
