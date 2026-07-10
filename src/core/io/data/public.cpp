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

    DataTable::DataTable(const Eigen::MatrixXd& d) : data(d), n_rows(static_cast<int>(data.rows())), n_cols(static_cast<int>(data.cols())) {}

    void DataTable::insert(int t, const Eigen::VectorXd& input) {
        if (input.cols() > 1) { 
            throw std::runtime_error("DataTable::insert Eigen::Matrix passed for 'input', expected Eigen::Vector"); 
        }
        if (input.rows() > n_cols - 1) { 
            throw std::runtime_error("DataTable::insert Number of rows in 'input' exceeds number of columns in DataTable"); 
        }
        if (t > n_rows - 1) { 
            throw std::runtime_error("DataTable::insert Input index 't' exceeds number of rows in DataTable"); 
        }

        data(t, 0) = t * constants::dt;
        Eigen::Index cols_to_copy = data.cols() - 1;
        data.block(t, 1, 1, cols_to_copy) = input.transpose(); // start_row, start_col, block_rows, block_cols
    }


    DataManager::DataManager(int tf, const runner::CLIFlags& cli_flags, const runner::JSONFlags& json_flags) 
        : cli_flags(cli_flags), json_flags(json_flags)
    {
        if (cli_flags.data_flag) {
            p_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            eul_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            w_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            v_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            u_surface_DT = DataTable{Eigen::MatrixXd::Zero(tf, 5+1) };
            u_propulsor_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            F_net_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            M_net_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            F_aero_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            M_aero_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            F_prop_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            M_prop_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };

            if (json_flags.control_flag) {
                eul_setpoint_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
                w_setpoint_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
                v_setpoint_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            }

            if (json_flags.avionics_flag) {
                p_meas_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
                eul_meas_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
                w_meas_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
                v_meas_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            }

            if (json_flags.estimation_flag) {
                p_est_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
                eul_est_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
                w_est_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
                v_est_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            }

            if (json_flags.wind_flag) {
                windB_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            }
        }
    }

    void DataManager::step(int t, const DataContext& context) {

        if (cli_flags.data_flag) {
            dynamics::EulerAngles eul_t;
            eul_t.set(context.Xt.q);

            p_DT->insert(t, context.Xt.p.data);
            eul_DT->insert(t, eul_t.data);
            w_DT->insert(t, context.Xt.w.data);
            v_DT->insert(t, context.Xt.v.data);
            u_surface_DT->insert(t, actuators::unpack_surface_actuator_inputs(context.u_surface));
            u_propulsor_DT->insert(t, actuators::unpack_propulsor_actuator_inputs(context.u_propulsor));
            F_net_DT->insert(t, context.WB_net.F.data);
            M_net_DT->insert(t, context.WB_net.M.data);
            F_aero_DT->insert(t, context.WB_aerodynamic.F.data);
            M_aero_DT->insert(t, context.WB_aerodynamic.M.data);
            F_prop_DT->insert(t, context.WB_propulsive.F.data);
            M_prop_DT->insert(t, context.WB_propulsive.M.data);

            if (json_flags.control_flag) {
                eul_setpoint_DT->insert(t, context.setpoint.eulIB.data);
                w_setpoint_DT->insert(t, context.setpoint.wB_BI.data);
                v_setpoint_DT->insert(t, context.setpoint.vB_BI.data);
            }

            if (json_flags.avionics_flag) {
                dynamics::EulerAngles eul_meas_t;
                eul_meas_t.set(context.Yt.q);
                p_meas_DT->insert(t, context.Yt.p.data);
                eul_meas_DT->insert(t, eul_meas_t.data);
                w_meas_DT->insert(t, context.Yt.w.data);
                v_meas_DT->insert(t, context.Yt.v.data);
            }

            if (json_flags.estimation_flag) {
                dynamics::EulerAngles eul_est_t;
                eul_est_t.set(context.Zt.q);
                p_est_DT->insert(t, context.Zt.p.data);
                eul_est_DT->insert(t, eul_est_t.data);
                w_est_DT->insert(t, context.Zt.w.data);
                v_est_DT->insert(t, context.Zt.v.data);
            }

            if (json_flags.wind_flag) {
                windB_DT->insert(t, context.windB.data);
            }
        }
    }


    void DataManager::save(const std::string& data_dir_path) {

        if (cli_flags.data_flag) {
            write_csv(p_DT->data, data_dir_path, "p");
            write_csv(eul_DT->data, data_dir_path, "eul");
            write_csv(w_DT->data, data_dir_path, "w");
            write_csv(v_DT->data, data_dir_path, "v");
            write_csv(u_surface_DT->data, data_dir_path, "u_surface");
            write_csv(u_propulsor_DT->data, data_dir_path, "u_propulsor");
            write_csv(F_net_DT->data, data_dir_path, "F_net");
            write_csv(M_net_DT->data, data_dir_path, "M_net");
            write_csv(F_aero_DT->data, data_dir_path, "F_aero");
            write_csv(M_aero_DT->data, data_dir_path, "M_aero");
            write_csv(F_prop_DT->data, data_dir_path, "F_prop");
            write_csv(M_prop_DT->data, data_dir_path, "M_prop");

            if (json_flags.control_flag) {
                write_csv(eul_setpoint_DT->data, data_dir_path, "eul_setpoint");
                write_csv(w_setpoint_DT->data, data_dir_path, "w_setpoint");
                write_csv(v_setpoint_DT->data, data_dir_path, "v_setpoint");
            }

            if (json_flags.avionics_flag) {
                write_csv(p_meas_DT->data, data_dir_path, "p_meas");
                write_csv(eul_meas_DT->data, data_dir_path, "eul_meas");
                write_csv(w_meas_DT->data, data_dir_path, "w_meas");
                write_csv(v_meas_DT->data, data_dir_path, "v_meas");
            }

            if (json_flags.estimation_flag) {
                write_csv(p_est_DT->data, data_dir_path, "p_est");
                write_csv(eul_est_DT->data, data_dir_path, "eul_est");
                write_csv(w_est_DT->data, data_dir_path, "w_est");
                write_csv(v_est_DT->data, data_dir_path, "v_est");
            }

            if (json_flags.wind_flag) {
                write_csv(windB_DT->data, data_dir_path, "windB");
            }
        }
    }
}
