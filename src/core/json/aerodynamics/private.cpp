#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/aerodynamics/private.hpp"
#include "core/json/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/util/validate/public.hpp"

namespace json {

    aerodynamics::DynamicDerivatives parse_dynamic_derivatives(const nlohmann::json& dyn_json) {
        return {
            .CL_qhat = dyn_json.value("CL_qhat", 0.0),
            .CD_qhat = dyn_json.value("CD_qhat", 0.0),
            .CM_qhat = dyn_json.value("CM_qhat", 0.0),
            .CL_phat = dyn_json.value("CL_phat", 0.0),
            .CD_phat = dyn_json.value("CD_phat", 0.0),
            .CM_phat = dyn_json.value("CM_phat", 0.0),
            .CL_rhat = dyn_json.value("CL_rhat", 0.0),
            .CD_rhat = dyn_json.value("CD_rhat", 0.0),
            .CM_rhat = dyn_json.value("CM_rhat", 0.0),
        };
    }

    aerodynamics::ControlDerivatives parse_control_derivatives(const nlohmann::json& ctrl_json) {
        return {
            .dCL_de = ctrl_json.value("dCL_de", 0.0),
            .dCM_de = ctrl_json.value("dCM_de", 0.0),
            .dCD_de = ctrl_json.value("dCD_de", 0.0),
            .dCL_da = ctrl_json.value("dCL_da", 0.0),
            .dCM_da = ctrl_json.value("dCM_da", 0.0),
            .dCD_da = ctrl_json.value("dCD_da", 0.0),
            .dCL_dr = ctrl_json.value("dCL_dr", 0.0),
            .dCM_dr = ctrl_json.value("dCM_dr", 0.0),
            .dCD_dr = ctrl_json.value("dCD_dr", 0.0),
            .dCL_df = ctrl_json.value("dCL_df", 0.0),
            .dCM_df = ctrl_json.value("dCM_df", 0.0),
            .dCD_df = ctrl_json.value("dCD_df", 0.0),
            .dCL_ds = ctrl_json.value("dCL_ds", 0.0),
            .dCM_ds = ctrl_json.value("dCM_ds", 0.0),
            .dCD_ds = ctrl_json.value("dCD_ds", 0.0),
        };
    }

    aerodynamics::AerodynamicProperties parse_aerodynamic_properties(const nlohmann::json& config, const structural::StructuralProperties& structural_properties) {
        const auto& surfaces_json = config.at("surfaces");
        if (!surfaces_json.is_array()) { 
            throw std::runtime_error("json::parse_aerodynamics_config expected 'surfaces' to be an array"); 
        }

        std::vector<aerodynamics::Surface> surfaces;
        surfaces.reserve(surfaces_json.size());
        for (const auto& surface_json : surfaces_json) {
            std::string id = surface_json.at("id").get<std::string>();
            Eigen::Vector3d p_ac = parse_Vector3d(surface_json.at("p_ac"));
            Eigen::Vector3d n = parse_Vector3d(surface_json.at("n"));
            util::validate_unit(n, "json::parse_aerodynamic_properties", id + " surface normal");

            surfaces.push_back(aerodynamics::Surface{
                .id = id,
                .chord = surface_json.at("chord").get<double>(),
                .span = surface_json.at("span").get<double>(),
                .p_ac_cg = p_ac - structural_properties.p_cg.data,
                .n = n,
                .CL0 = surface_json.at("CL0").get<double>(),
                .e = surface_json.at("e").get<double>(),
                .i = surface_json.at("i").get<double>(),
                .CD0 = surface_json.at("CD0").get<double>(),
                .CDa = surface_json.at("CDa").get<double>(),
                .a0 = surface_json.at("a0").get<double>(),
                .CM0 = surface_json.at("CM0").get<double>(),
                .CMa = surface_json.at("CMa").get<double>(),
                .dyn = surface_json.contains("dynamic_derivatives") ? parse_dynamic_derivatives(surface_json.at("dynamic_derivatives")) : aerodynamics::DynamicDerivatives{},
                .ctrl = surface_json.contains("control_derivatives") ? parse_control_derivatives(surface_json.at("control_derivatives")) : aerodynamics::ControlDerivatives{},
            });
        }
        return { surfaces };
    }

}
