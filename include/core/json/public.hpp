#pragma once
#include <filesystem>
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/actuators/public.hpp"
#include "core/json/aerodynamics/public.hpp"
#include "core/json/avionics/public.hpp"
#include "core/json/control/public.hpp"
#include "core/json/estimation/public.hpp"
#include "core/json/guidance/public.hpp"
#include "core/json/initialization/public.hpp"
#include "core/json/operating/public.hpp"
#include "core/json/structural/public.hpp"

namespace json {

    Eigen::Vector3d parse_Vector3d(const nlohmann::json& values);
    Eigen::Vector4d parse_Vector4d(const nlohmann::json& values);
    Eigen::Matrix3d parse_Matrix3d(const nlohmann::json& values);
    Eigen::Matrix4d parse_Matrix4d(const nlohmann::json& values);
    Eigen::MatrixXd parse_MatrixXd(const nlohmann::json& values);
    Eigen::Quaterniond parse_Quaterniond(const nlohmann::json& values);

    nlohmann::json read_json_file(const std::filesystem::path& path);
    std::filesystem::path resolve_run_config_entry_path(const std::string& key);

    void dump_configs(const std::string& dir_path);
}
