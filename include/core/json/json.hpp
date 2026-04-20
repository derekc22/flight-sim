#pragma once
#include <filesystem>
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/aerodynamics.hpp"
#include "core/json/actuators.hpp"
#include "core/json/control.hpp"
#include "core/json/initialization.hpp"
#include "core/json/operating.hpp"
#include "core/json/structural.hpp"
#include "core/json/avionics.hpp"

namespace json {

    Eigen::Vector3d parse_Vector3d(const nlohmann::json& values);
    Eigen::Vector4d parse_Vector4d(const nlohmann::json& values);
    Eigen::Matrix3d parse_Matrix3d(const nlohmann::json& values);
    Eigen::Matrix4d parse_Matrix4d(const nlohmann::json& values);
    Eigen::MatrixXd parse_MatrixXd(const nlohmann::json& values);
    Eigen::Quaterniond parse_Quaterniond(const nlohmann::json& values);

    nlohmann::json read_json_file(const std::filesystem::path& path);
    std::filesystem::path resolve_config_path(const std::filesystem::path& run_path, const std::string& config_path);
    std::filesystem::path resolve_run_config_entry_path(const std::string& key);

    void write_json(const nlohmann::json& config, const std::string& dir, const std::string& fname);

    void dump_configs(const std::string& dir);
}
