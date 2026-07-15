#pragma once
#include <filesystem>
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

namespace json {

    Eigen::Vector3d parse_Vector3d(const nlohmann::json& values);
    Eigen::Vector4d parse_Vector4d(const nlohmann::json& values);
    Eigen::Matrix3d parse_Matrix3d(const nlohmann::json& values);
    Eigen::Matrix4d parse_Matrix4d(const nlohmann::json& values);
    Eigen::MatrixXd parse_MatrixXd(const nlohmann::json& values);
    Eigen::Quaterniond parse_Quaterniond(const nlohmann::json& values);

    nlohmann::json read_json_file(const std::filesystem::path& path);
    std::filesystem::path resolve_run_config_entry_path(const std::string& key);

    void dump_run_configs(const std::string& dir_path);
    void dump_analyze_configs(const std::string& dir_path);

}
