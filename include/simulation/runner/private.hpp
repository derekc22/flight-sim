#pragma once
#include <chrono>
#include <string>
#include <Eigen/Dense>
#include <spdlog/spdlog.h>
#include "simulation/runner/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/vehicles/public.hpp"
#include "simulation/failures/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/sensors/public.hpp"
#include "simulation/avionics/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/control/public.hpp"
#include "core/messages/public.hpp"
#include "core/connection/public.hpp"
#include "core/io/analysis/public.hpp"
#include "core/io/data/public.hpp"
#include "core/io/rerun/public.hpp"

namespace runner {

    struct CLIOptions {
        std::string aircraft_id;
        CLIFlags flags;
        std::string log_dir_path;
        std::string data_dir_path;
        std::string report_dir_path;
    };

    struct Scheduler {
        ModuleRates module_hz;

        double sensor_tick = constants::hz;
        double avionics_tick = constants::hz;
        double estimation_tick = constants::hz;
        double guidance_tick = constants::hz;
        double control_tick = constants::hz;
        double log_tick = constants::hz;

        int sensor_elapsed_ticks = 1;
        int avionics_elapsed_ticks = 1;
        int estimation_elapsed_ticks = 1;
        int control_elapsed_ticks = 1;

        int guidance_kf; // total number of guidance steps

        Scheduler(const ModuleRates& module_hz, int tf);
        void step();
    };

    vehicles::Aircraft load_vehicle(const std::string& aircraft_id, const JSONFlags& json_flags);

    struct RunManager {
        CLIOptions cli_options;
        JSONOptions json_options;
        vehicles::Aircraft aircraft;
        io::DataManager data_manager;
        io::RerunManager rerun_manager;
        io::AnalysisManager analysis_manager;
        failures::FailureManager failure_manager;

        actuators::SurfaceActuatorInputs_T<double> u_surface_actual_prev{};
        actuators::PropulsorActuatorInputs_T<double> u_propulsor_actual_prev{};

        // initialize trim and linearization solutions
        trim::TrimSolution trim_sol;
        linearization::LocalLinearization lin_sol;

        // initialize prior-step net wrench
        dynamics::Wrench WB_net_t_1{ 
            .F = dynamics::Force { constants::Zero3 }, 
            .M = dynamics::Moment{ constants::Zero3 } 
        };

        // initialize prior-tick values to perform ZOH
        sensors::SensorMeasurements sensor_meas_t_1;
        dynamics::RigidBodyState Yt_1;
        dynamics::RigidBodyState Zt_1;
        guidance::GuidanceSetpoint setpoint_t_1;
        control::ControlOutput u_cmd_t_1;

        // initialize udp out cache
        messages::ProcessedFlightGearMessageOut cached_msg_out{};

        connection::UDPOut udp_out;
        connection::UDPIn udp_in;
        std::chrono::steady_clock::time_point next;

        // initialize scheduler
        Scheduler scheduler;

        RunManager(const CLIOptions& cli_options, const JSONOptions& json_options);
        ~RunManager();

        void cleanup();
        void run();
        void step(int t);
    };

    std::string print_vec(const char* name, const Eigen::Vector3d& x, const char* unit);

    void log_state(
        int t,
        const dynamics::RigidBodyState& Xt,
        const geography::GeographicState& geo,
        const aerodynamics::AerodynamicState& aero,
        const atmospheric::Wind& windB
    );

}
