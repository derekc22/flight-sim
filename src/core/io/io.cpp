#include <iomanip>
#include <filesystem>
#include <fstream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "core/io/io.hpp"

namespace io {

    void createDir(const std::string& dir) {
        std::filesystem::create_directories(dir);
    }

    void saveVectorToFile(std::vector<int>& data, std::string fname){
        std::string path_name = "data/" + fname + ".csv";
        std::ofstream file_v(path_name);
        for (size_t i = 0; i < data.size(); i++){
            file_v << data[i];
            if (i < data.size() - 1) file_v << ",";  // comma delimiter
        }
        file_v.close();
    }

    // void save(const Eigen::MatrixXd& DM, const std::string& dir, const std::string& fname) {
    //     std::string path_name = dir + "/" + fname + ".csv";
    //     std::ofstream file_m(path_name);

    //     for (int i = 0; i < M.rows(); ++i) {
    //         for (int j = 0; j < M.cols(); ++j) {
    //             file_m << M(i, j);
    //             if (j < M.cols() - 1) file_m << ","; // comma delimiter
    //         }
    //         file_m << "\n";
    //     }
    //     file_m.close();
    // }

    DataMatrix::DataMatrix(const Eigen::MatrixXd& d) 
        : data(d), 
        n_rows(static_cast<int>(data.rows())), 
        n_cols(static_cast<int>(data.cols())) {};

    void DataMatrix::write_csv(const std::string& dir, const std::string& fname) const {

        createDir(dir);

        // std::string path_name = dir + "/" + fname + ".csv";
        auto path_name = std::filesystem::path(dir) / (fname + ".csv");

        std::ofstream file_m(path_name);
        if (!file_m.is_open()) {
            throw std::runtime_error("Failed to open file: " + path_name.string());
        }

        for (int i = 0; i < n_rows; ++i) {
            for (int j = 0; j < n_cols; ++j) {
                file_m << data(i, j);
                if (j < n_cols - 1) file_m << ","; // comma delimiter
            }
            file_m << "\n";
        }
        file_m.close();
    }

    void DataMatrix::set(int t, const Eigen::VectorXd input, double dt){
        if (input.cols() > 1) { throw std::runtime_error("io::DataMatrix::set Eigen::Matrix passed for 'input', expected Eigen::Vector"); }
        if (input.rows() > n_cols-1) { throw std::runtime_error("io::DataMatrix::set Number of rows in 'input' exceeds number of columns in DataMatrix"); }
        if (t > n_rows-1) { throw std::runtime_error("io::DataMatrix::set Input index 't' exceeds number of rows in DataMatrix"); }

        data(t, 0) = t * dt;
        Eigen::Index cols_to_copy = data.cols() - 1;
        data.block(t, 1, 1, cols_to_copy) = input.transpose(); // startRow, startCol, blockRows, blockCols.
    }



}
