#include "core/io/data/public/manager.hpp"

#include "core/io/public/files.hpp"
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/runner/public/data/types.hpp"

namespace io
{

	DataManager::DataManager(
		int tf,
		const runner::JSONFlags& json_flags)
		: json_flags(json_flags)
	{
		p_DM = DataMatrix(tf, 3 + 1);
		eul_DM = DataMatrix(tf, 3 + 1);
		w_DM = DataMatrix(tf, 3 + 1);
		v_DM = DataMatrix(tf, 3 + 1);
		u_surface_DM = DataMatrix(tf, 5 + 1);
		u_propulsor_DM = DataMatrix(tf, 3 + 1);
		u_surface_commanded_DM = DataMatrix(tf, 5 + 1);
		u_propulsor_commanded_DM = DataMatrix(tf, 3 + 1);
		F_net_DM = DataMatrix(tf, 3 + 1);
		M_net_DM = DataMatrix(tf, 3 + 1);
		F_aerodynamic_DM = DataMatrix(tf, 3 + 1);
		M_aerodynamic_DM = DataMatrix(tf, 3 + 1);
		F_propulsive_DM = DataMatrix(tf, 3 + 1);
		M_propulsive_DM = DataMatrix(tf, 3 + 1);

		if (json_flags.control_flag) {
			eul_setpoint_DM = DataMatrix(tf, 3 + 1);
			w_setpoint_DM = DataMatrix(tf, 3 + 1);
			v_setpoint_DM = DataMatrix(tf, 3 + 1);
		}

		if (json_flags.avionics_flag) {
			p_measured_DM = DataMatrix(tf, 3 + 1);
			eul_measured_DM = DataMatrix(tf, 3 + 1);
			w_measured_DM = DataMatrix(tf, 3 + 1);
			v_measured_DM = DataMatrix(tf, 3 + 1);
		}

		if (json_flags.estimation_flag) {
			p_estimated_DM = DataMatrix(tf, 3 + 1);
			eul_estimated_DM = DataMatrix(tf, 3 + 1);
			w_estimated_DM = DataMatrix(tf, 3 + 1);
			v_estimated_DM = DataMatrix(tf, 3 + 1);
		}

		if (json_flags.wind_flag) {
			windB_DM = DataMatrix(tf, 3 + 1);
		}
	}

	void DataManager::step(
		const DataManagerInput& input)
	{
		int t = input.t;
		const DataContext& context = input.context;

		dynamics::EulerAngles eul_t;
		eul_t.set(context.Xt.q);

		p_DM->insert(t, context.Xt.p.data);
		eul_DM->insert(t, eul_t.data);
		w_DM->insert(t, context.Xt.w.data);
		v_DM->insert(t, context.Xt.v.data);
		u_surface_DM->insert(t, actuators::unpack_surface_actuator_inputs(context.u_surface));
		u_propulsor_DM->insert(t, actuators::unpack_propulsor_actuator_inputs(context.u_propulsor));
		u_surface_commanded_DM->insert(t, actuators::unpack_surface_actuator_inputs(context.u_surface_commanded));
		u_propulsor_commanded_DM->insert(t, actuators::unpack_propulsor_actuator_inputs(context.u_propulsor_commanded));
		F_net_DM->insert(t, context.WB_net.F.data);
		M_net_DM->insert(t, context.WB_net.M.data);
		F_aerodynamic_DM->insert(t, context.WB_aerodynamic.F.data);
		M_aerodynamic_DM->insert(t, context.WB_aerodynamic.M.data);
		F_propulsive_DM->insert(t, context.WB_propulsive.F.data);
		M_propulsive_DM->insert(t, context.WB_propulsive.M.data);

		if (json_flags.control_flag) {
			eul_setpoint_DM->insert(t, context.setpoint.eulIB.data);
			w_setpoint_DM->insert(t, context.setpoint.wB_BI.data);
			v_setpoint_DM->insert(t, context.setpoint.vB_BI.data);
		}

		if (json_flags.avionics_flag) {
			dynamics::EulerAngles eul_measured_t;
			eul_measured_t.set(context.Yt.q);
			p_measured_DM->insert(t, context.Yt.p.data);
			eul_measured_DM->insert(t, eul_measured_t.data);
			w_measured_DM->insert(t, context.Yt.w.data);
			v_measured_DM->insert(t, context.Yt.v.data);
		}

		if (json_flags.estimation_flag) {
			dynamics::EulerAngles eul_estimated_t;
			eul_estimated_t.set(context.Zt.q);
			p_estimated_DM->insert(t, context.Zt.p.data);
			eul_estimated_DM->insert(t, eul_estimated_t.data);
			w_estimated_DM->insert(t, context.Zt.w.data);
			v_estimated_DM->insert(t, context.Zt.v.data);
		}

		if (json_flags.wind_flag) {
			windB_DM->insert(t, context.windB.data);
		}
	}

	void DataManager::save(
		const std::string& data_dir_path)
	{

		write_csv(p_DM->data, data_dir_path, "p");
		write_csv(eul_DM->data, data_dir_path, "eul");
		write_csv(w_DM->data, data_dir_path, "w");
		write_csv(v_DM->data, data_dir_path, "v");
		write_csv(u_surface_DM->data, data_dir_path, "u_surface");
		write_csv(u_propulsor_DM->data, data_dir_path, "u_propulsor");
		write_csv(u_surface_commanded_DM->data, data_dir_path, "u_surface_commanded");
		write_csv(u_propulsor_commanded_DM->data, data_dir_path, "u_propulsor_commanded");
		write_csv(F_net_DM->data, data_dir_path, "F_net");
		write_csv(M_net_DM->data, data_dir_path, "M_net");
		write_csv(F_aerodynamic_DM->data, data_dir_path, "F_aerodynamic");
		write_csv(M_aerodynamic_DM->data, data_dir_path, "M_aerodynamic");
		write_csv(F_propulsive_DM->data, data_dir_path, "F_propulsive");
		write_csv(M_propulsive_DM->data, data_dir_path, "M_propulsive");

		if (json_flags.control_flag) {
			write_csv(eul_setpoint_DM->data, data_dir_path, "eul_setpoint");
			write_csv(w_setpoint_DM->data, data_dir_path, "w_setpoint");
			write_csv(v_setpoint_DM->data, data_dir_path, "v_setpoint");
		}

		if (json_flags.avionics_flag) {
			write_csv(p_measured_DM->data, data_dir_path, "p_measured");
			write_csv(eul_measured_DM->data, data_dir_path, "eul_measured");
			write_csv(w_measured_DM->data, data_dir_path, "w_measured");
			write_csv(v_measured_DM->data, data_dir_path, "v_measured");
		}

		if (json_flags.estimation_flag) {
			write_csv(p_estimated_DM->data, data_dir_path, "p_estimated");
			write_csv(eul_estimated_DM->data, data_dir_path, "eul_estimated");
			write_csv(w_estimated_DM->data, data_dir_path, "w_estimated");
			write_csv(v_estimated_DM->data, data_dir_path, "v_estimated");
		}

		if (json_flags.wind_flag) {
			write_csv(windB_DM->data, data_dir_path, "windB");
		}
	}
} // namespace io
