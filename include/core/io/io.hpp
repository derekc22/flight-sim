#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>

namespace io {

    void create_dir(const std::string& dir);
    void write_txt(const std::string& text, const std::string& dir, const std::string& fname);
    void write_csv(const Eigen::MatrixXd& data, const std::string& dir, const std::string& fname);
    void save_vector_to_file(std::vector<int>& data, std::string fname);

    struct DataMatrix {
        Eigen::MatrixXd data;
        int n_rows;
        int n_cols;

        DataMatrix(const Eigen::MatrixXd& d);
        void insert(int t, const Eigen::VectorXd data);

    };

    std::string get_datetime();
}
