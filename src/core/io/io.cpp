#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>
#include "core/io/io.hpp"
#include "simulation/actuators/propulsor/propulsor.hpp"
#include "simulation/actuators/surface/surface.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/dynamics/shared.hpp"

namespace io {

    void create_dir(const std::string& dir) {
        std::filesystem::create_directories(dir);
    }

    void write_txt(const std::string& text, const std::string& dir, const std::string& fname) {
        create_dir(dir);

        const auto path_name = std::filesystem::path(dir) / (fname + ".txt");
        std::ofstream file_txt(path_name);
        if (!file_txt.is_open()) { throw std::runtime_error("Failed to open file: " + path_name.string()); }

        file_txt << text;
        file_txt.close();
        std::cout << "File saved successfully to " << path_name.string() << std::endl;
    }

    void write_csv(const Eigen::MatrixXd& data, const std::string& dir, const std::string& fname) {
        create_dir(dir);

        const auto path_name = std::filesystem::path(dir) / (fname + ".csv");
        std::ofstream file_csv(path_name);
        if (!file_csv.is_open()) { throw std::runtime_error("Failed to open file: " + path_name.string()); }

        for (Eigen::Index i = 0; i < data.rows(); ++i) {
            for (Eigen::Index j = 0; j < data.cols(); ++j) {
                file_csv << data(i, j);
                if (j < data.cols() - 1) file_csv << ",";
            }
            file_csv << "\n";
        }

        file_csv.close();
        std::cout << "File saved successfully to " << path_name.string() << std::endl;
    }

    void save_vector_to_file(const std::vector<int>& data, const std::string& fname){
        std::string path_name = "data/" + fname + ".csv";
        std::ofstream file_v(path_name);
        for (size_t i = 0; i < data.size(); i++){
            file_v << data[i];
            if (i < data.size() - 1) file_v << ",";  // comma delimiter
        }
        file_v.close();
    }

    DataTable::DataTable(const Eigen::MatrixXd& d) : data(d), n_rows(static_cast<int>(data.rows())), n_cols(static_cast<int>(data.cols())) {}

    void DataTable::insert(int t, const Eigen::VectorXd& input) {
        if (input.cols() > 1) { throw std::runtime_error("DataTablensert Eigen::Matrix passed for 'input', expected Eigen::Vector"); }
        if (input.rows() > n_cols - 1) { throw std::runtime_error("DataTablensert Number of rows in 'input' exceeds number of columns in DataTable"); }
        if (t > n_rows - 1) { throw std::runtime_error("DataTablensert Input index 't' exceeds number of rows in DataTable"); }

        data(t, 0) = t * constants::dt;
        Eigen::Index cols_to_copy = data.cols() - 1;
        data.block(t, 1, 1, cols_to_copy) = input.transpose(); // startRow, startCol, blockRows, blockCols.
    }


    DataManager::DataManager(int tf, bool data_bool, bool control_bool, bool sensor_bool, bool estimation_bool)
        : data_bool(data_bool), control_bool(control_bool), sensor_bool(sensor_bool),  estimation_bool(estimation_bool)
    {
        if (data_bool) {
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

            if (control_bool) {
                eul_setpoint_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
                w_setpoint_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
                v_setpoint_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            }

            if (sensor_bool) {
                p_meas_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
                eul_meas_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
                w_meas_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
                v_meas_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            }

            if (estimation_bool) {
                p_est_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
                eul_est_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
                w_est_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
                v_est_DT = DataTable{Eigen::MatrixXd::Zero(tf, 3+1) };
            }
        }
    }

    void DataManager::step(int t, const DataContext& context) {
        if (data_bool) {
            dynamics::EulerAngles eul_t;
            eul_t.set(context.xN_t.q);

            p_DT->insert(t, context.xN_t.p.data);
            eul_DT->insert(t, eul_t.data);
            w_DT->insert(t, context.xN_t.w.data);
            v_DT->insert(t, context.xN_t.v.data);
            u_surface_DT->insert(t, actuators::unpack_full_surface_actuator_inputs(context.u_surface_actual));
            u_propulsor_DT->insert(t, actuators::unpack_full_propulsor_actuator_inputs(context.u_propulsor_actual));
            F_net_DT->insert(t, context.WB_net.F.data);
            M_net_DT->insert(t, context.WB_net.M.data);
            F_aero_DT->insert(t, context.WB_aero.F.data);
            M_aero_DT->insert(t, context.WB_aero.M.data);
            F_prop_DT->insert(t, context.WB_propulsive.F.data);
            M_prop_DT->insert(t, context.WB_propulsive.M.data);

            if (control_bool) {
                eul_setpoint_DT->insert(t, context.setpoint.eulIB.data);
                w_setpoint_DT->insert(t, context.setpoint.wB_BI.data);
                v_setpoint_DT->insert(t, context.setpoint.vB_BI.data);
            }

            if (sensor_bool) {
                dynamics::EulerAngles eul_meas_t;
                eul_meas_t.set(context.yN_t.q);
                p_meas_DT->insert(t, context.yN_t.p.data);
                eul_meas_DT->insert(t, eul_meas_t.data);
                w_meas_DT->insert(t, context.yN_t.w.data);
                v_meas_DT->insert(t, context.yN_t.v.data);
            }

            if (estimation_bool) {
                dynamics::EulerAngles eul_est_t;
                eul_est_t.set(context.zN_t.q);
                p_est_DT->insert(t, context.zN_t.p.data);
                eul_est_DT->insert(t, eul_est_t.data);
                w_est_DT->insert(t, context.zN_t.w.data);
                v_est_DT->insert(t, context.zN_t.v.data);
            }
        }
    }


    void DataManager::save(const std::string& out_dir_path) {

        if (data_bool){
            write_csv(p_DT->data, out_dir_path, "p");
            write_csv(eul_DT->data, out_dir_path, "eul");
            write_csv(w_DT->data, out_dir_path, "w");
            write_csv(v_DT->data, out_dir_path, "v");
            write_csv(u_surface_DT->data, out_dir_path, "u_surface");
            write_csv(u_propulsor_DT->data, out_dir_path, "u_propulsor");
            write_csv(F_net_DT->data, out_dir_path, "F_net");
            write_csv(M_net_DT->data, out_dir_path, "M_net");
            write_csv(F_aero_DT->data, out_dir_path, "F_aero");
            write_csv(M_aero_DT->data, out_dir_path, "M_aero");
            write_csv(F_prop_DT->data, out_dir_path, "F_prop");
            write_csv(M_prop_DT->data, out_dir_path, "M_prop");

            if (control_bool) {
                write_csv(eul_setpoint_DT->data, out_dir_path, "eul_setpoint");
                write_csv(w_setpoint_DT->data, out_dir_path, "w_setpoint");
                write_csv(v_setpoint_DT->data, out_dir_path, "v_setpoint");
            }

            if (sensor_bool) {
                write_csv(p_meas_DT->data, out_dir_path, "p_meas");
                write_csv(eul_meas_DT->data, out_dir_path, "eul_meas");
                write_csv(w_meas_DT->data, out_dir_path, "w_meas");
                write_csv(v_meas_DT->data, out_dir_path, "v_meas");
            }

            if (estimation_bool) {
                write_csv(p_est_DT->data, out_dir_path, "p_est");
                write_csv(eul_est_DT->data, out_dir_path, "eul_est");
                write_csv(w_est_DT->data, out_dir_path, "w_est");
                write_csv(v_est_DT->data, out_dir_path, "v_est");
            }
        }
    }



    std::string get_datetime() {
        std::time_t std_tm = std::time(nullptr);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y%b%d_%H-%M-%S", std::localtime(&std_tm));
        std::string dtn = buf;
        return dtn;
    }

}
