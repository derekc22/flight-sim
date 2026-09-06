#pragma once
#include "core/io/data/public/data/types.hpp"
#include "simulation/runner/public/data/types.hpp"

#include <optional>
#include <string>

namespace io
{

	struct DataRecorder {
		runner::JSONFlags json_flags;

		std::optional<DataMatrix> p_DM;
		std::optional<DataMatrix> p_measured_DM;
		std::optional<DataMatrix> p_estimated_DM;
		std::optional<DataMatrix> eul_DM;
		std::optional<DataMatrix> eul_measured_DM;
		std::optional<DataMatrix> eul_estimated_DM;
		std::optional<DataMatrix> eul_setpoint_DM;
		std::optional<DataMatrix> w_DM;
		std::optional<DataMatrix> w_measured_DM;
		std::optional<DataMatrix> w_estimated_DM;
		std::optional<DataMatrix> w_setpoint_DM;
		std::optional<DataMatrix> v_DM;
		std::optional<DataMatrix> v_measured_DM;
		std::optional<DataMatrix> v_estimated_DM;
		std::optional<DataMatrix> v_setpoint_DM;
		std::optional<DataMatrix> u_surface_DM;
		std::optional<DataMatrix> u_propulsor_DM;
		std::optional<DataMatrix> u_surface_commanded_DM;
		std::optional<DataMatrix> u_propulsor_commanded_DM;
		std::optional<DataMatrix> F_net_DM;
		std::optional<DataMatrix> M_net_DM;
		std::optional<DataMatrix> F_aerodynamic_DM;
		std::optional<DataMatrix> M_aerodynamic_DM;
		std::optional<DataMatrix> F_propulsive_DM;
		std::optional<DataMatrix> M_propulsive_DM;
		std::optional<DataMatrix> windB_DM;

		void step(const DataRecorderInput& input);
		void save(const std::string& data_dir_path);
		DataRecorder(int tf, const runner::JSONFlags& json_flags);
	};

} // namespace io
