#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>
#include "core/io/io.hpp"
#include "simulation/constants/constants.hpp"

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

    void save_vector_to_file(std::vector<int>& data, std::string fname){
        std::string path_name = "data/" + fname + ".csv";
        std::ofstream file_v(path_name);
        for (size_t i = 0; i < data.size(); i++){
            file_v << data[i];
            if (i < data.size() - 1) file_v << ",";  // comma delimiter
        }
        file_v.close();
    }

    DataMatrix::DataMatrix(const Eigen::MatrixXd& d) : data(d), n_rows(static_cast<int>(data.rows())), n_cols(static_cast<int>(data.cols())) {}

    void DataMatrix::insert(int t, const Eigen::VectorXd input) {
        if (input.cols() > 1) { throw std::runtime_error("io::DataMatrix::insert Eigen::Matrix passed for 'input', expected Eigen::Vector"); }
        if (input.rows() > n_cols - 1) { throw std::runtime_error("io::DataMatrix::insert Number of rows in 'input' exceeds number of columns in DataMatrix"); }
        if (t > n_rows - 1) { throw std::runtime_error("io::DataMatrix::insert Input index 't' exceeds number of rows in DataMatrix"); }

        data(t, 0) = t * constants::dt;
        Eigen::Index cols_to_copy = data.cols() - 1;
        data.block(t, 1, 1, cols_to_copy) = input.transpose(); // startRow, startCol, blockRows, blockCols.
    }


    std::string get_datetime() {
        std::time_t std_tm = std::time(nullptr);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y%b%d_%H-%M-%S", std::localtime(&std_tm));
        std::string dtn = buf;
        return dtn;
    }

}
