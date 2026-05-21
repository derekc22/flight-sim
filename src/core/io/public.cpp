#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include "core/io/public.hpp"
#include "core/io/private.hpp"

namespace io {

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
}
