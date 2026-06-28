#pragma once
#include <chrono>
#include <string>
#include "core/connection/public.hpp"
#include "core/io/analysis/public.hpp"
#include "core/io/data/public.hpp"
#include "core/io/rerun/public.hpp"
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

namespace runner {

    struct CLIOptions {
        std::string aircraft_id;
        bool data_bool;
        bool analysis_bool;
        std::string log_dir_path;
        std::string data_dir_path;
        std::string report_dir_path;
    };

    struct JSONOptions {
        int tf;
        double avionics_hz;
        double estimation_hz;
        double guidance_hz;
        double control_hz;

        bool trim_bool;
        bool avionics_bool;
        bool control_bool;
        bool estimation_bool;
        bool wind_bool;
        bool verbose_bool;
        bool rerun_bool;
    };

    struct MultiRateAccumulator {
        double avionics_acc = constants::hz;
        double estimation_acc = constants::hz;
        double guidance_acc = constants::hz;
        double control_acc = constants::hz;
        void step(const JSONOptions& json_options);
    };

    vehicles::Aircraft load_vehicle(const std::string& aircraft_id, bool trim_bool);

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
        dynamics::RigidBodyState Yt_1;
        dynamics::RigidBodyState Zt_1;
        guidance::GuidanceSetpoint setpoint_t_1;
        control::ControlOutput u_cmd_t_1;

        // initialize udp out cache
        messages::ProcessedFlightGearMessageOut cached_msg_out{};

        connection::UDPOut udp_out;
        connection::UDPIn udp_in;
        std::chrono::steady_clock::time_point next;

        // initialize accumulator
        MultiRateAccumulator acc{};

        RunManager(CLIOptions cli_options, JSONOptions json_options);
        ~RunManager();

        void cleanup();
        void run();
        void step(int t);
    };

}
