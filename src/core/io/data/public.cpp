#include <stdexcept>
#include <Eigen/Dense>
#include "core/io/data/public.hpp"
#include "core/io/public.hpp"
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/runner/public.hpp"

namespace io {

    DataMatrix::DataMatrix(int n_rows, int n_cols) : 
        n_rows(n_rows), 
        n_cols(n_cols), 
        data(Eigen::MatrixXd::Zero(n_rows, n_cols)) 
    {}

    void DataMatrix::insert(int t, const Eigen::VectorXd& input) {
        if (input.cols() > 1) { 
            throw std::runtime_error("DataMatrix::insert Eigen::Matrix passed for 'input', expected Eigen::Vector"); 
        }
        if (input.rows() > n_cols - 1) { 
            throw std::runtime_error("DataMatrix::insert Number of rows in 'input' exceeds number of columns in DataMatrix"); 
        }
        if (t > n_rows - 1) { 
            throw std::runtime_error("DataMatrix::insert Input index 't' exceeds number of rows in DataMatrix"); 
        }

        data(t, 0) = t * constants::dt;
        Eigen::Index cols_to_copy = data.cols() - 1;
        data.block(t, 1, 1, cols_to_copy) = input.transpose(); // start_row, start_col, block_rows, block_cols
    }


    DataManager::DataManager(int tf, const runner::JSONFlags& json_flags) :
        json_flags(json_flags)
    {
        p_DM = DataMatrix(tf, 3+1);
        eul_DM = DataMatrix(tf, 3+1);
        w_DM = DataMatrix(tf, 3+1);
        v_DM = DataMatrix(tf, 3+1);
        u_surface_DM = DataMatrix(tf, 5+1);
        u_propulsor_DM = DataMatrix(tf, 3+1);
        u_surface_commanded_DM = DataMatrix(tf, 5+1);
        u_propulsor_commanded_DM = DataMatrix(tf, 3+1);
        F_net_DM = DataMatrix(tf, 3+1);
        M_net_DM = DataMatrix(tf, 3+1);
        F_aerodynamic_DM = DataMatrix(tf, 3+1);
        M_aerodynamic_DM = DataMatrix(tf, 3+1);
        F_propulsive_DM = DataMatrix(tf, 3+1);
        M_propulsive_DM = DataMatrix(tf, 3+1);

        if (json_flags.control_flag) {
            eul_setpoint_DM = DataMatrix(tf, 3+1);
            w_setpoint_DM = DataMatrix(tf, 3+1);
            v_setpoint_DM = DataMatrix(tf, 3+1);
            allocator_diagnostics_DM = DataMatrix(tf, 34+1);
        }

        if (json_flags.avionics_flag) {
            p_measured_DM = DataMatrix(tf, 3+1);
            eul_measured_DM = DataMatrix(tf, 3+1);
            w_measured_DM = DataMatrix(tf, 3+1);
            v_measured_DM = DataMatrix(tf, 3+1);
        }

        if (json_flags.estimation_flag) {
            p_estimated_DM = DataMatrix(tf, 3+1);
            eul_estimated_DM = DataMatrix(tf, 3+1);
            w_estimated_DM = DataMatrix(tf, 3+1);
            v_estimated_DM = DataMatrix(tf, 3+1);
        }

        if (json_flags.wind_flag) {
            windB_DM = DataMatrix(tf, 3+1);
        }
    }

    void DataManager::step(int t, const DataContext& context) {

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
            Eigen::VectorXd allocator_diagnostics(34);
            allocator_diagnostics.segment<6>(0) = context.allocator_diagnostics.mu_requested;
            allocator_diagnostics.segment<6>(6) = context.allocator_diagnostics.mu_baseline;
            allocator_diagnostics.segment<6>(12) = context.allocator_diagnostics.mu_predicted;
            allocator_diagnostics.segment<6>(18) = dynamics::unpack_wrench(context.WB_net);
            allocator_diagnostics.segment<6>(24) = context.allocator_diagnostics.u_commanded;
            allocator_diagnostics(30) = context.allocator_diagnostics.tracking_cost;
            allocator_diagnostics(31) = context.allocator_diagnostics.movement_cost;
            allocator_diagnostics(32) = context.allocator_diagnostics.trim_cost;
            allocator_diagnostics(33) = context.allocator_diagnostics.allocation_limited;
            allocator_diagnostics_DM->insert(t, allocator_diagnostics);
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


    void DataManager::save(const std::string& data_dir_path) {

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
            write_csv(allocator_diagnostics_DM->data, data_dir_path, "allocator_diagnostics");
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
}
