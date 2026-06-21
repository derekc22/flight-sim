#pragma once
#include <chrono>
#include <string>
#include "core/connection/public.hpp"
#include "core/io/analysis/public.hpp"
#include "core/io/data/public.hpp"
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/vehicles/public.hpp"

namespace runner {

    struct SimulationOptions {
        std::string aircraft_id;
        int tf;
        bool trim_bool;
        bool sensor_bool;
        bool control_bool;
        bool estimation_bool;
        bool wind_bool;
        bool verbose_bool;
        bool data_bool;
        std::string data_dir_path;
        std::string report_dir_path;
        bool analysis_bool;
    };

    vehicles::Aircraft load(const std::string& aircraft_id, bool trim_bool);

    struct SimulationRunner {
        SimulationOptions options;
        vehicles::Aircraft aircraft;
        io::DataManager data_manager;
        io::AnalysisManager analysis_manager;

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

        // initialize udp out cache
        messages::ProcessedFlightGearMessageOut cached_msg_out{};

       connection::UDPOut udp_out;
        connection::UDPIn udp_in;
        std::chrono::steady_clock::time_point next;

        SimulationRunner(SimulationOptions options);
        ~SimulationRunner();

        void cleanup();
        void run();
        void step(int t);
    };

}
