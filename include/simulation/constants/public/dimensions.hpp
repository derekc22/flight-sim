#pragma once
#include <cstddef>

namespace constants
{

	// Modeled state and input dimensions
	inline constexpr std::size_t state_dim = 8;

	inline constexpr std::size_t surface_input_dim = 3;
	inline constexpr std::size_t fixed_surface_input_dim = 2;

	inline constexpr std::size_t propulsor_input_dim = 3;
	inline constexpr std::size_t fixed_propulsor_input_dim = 0;

	inline constexpr std::size_t full_surface_input_dim = surface_input_dim + fixed_surface_input_dim;
	inline constexpr std::size_t full_propulsor_input_dim = propulsor_input_dim + fixed_propulsor_input_dim;

	inline constexpr std::size_t fixed_input_dim = fixed_surface_input_dim + fixed_propulsor_input_dim;
	inline constexpr std::size_t input_dim = surface_input_dim + propulsor_input_dim;
	inline constexpr std::size_t full_input_dim = input_dim + fixed_input_dim;

	inline constexpr std::size_t state_input_dim = state_dim + input_dim;

	inline constexpr std::size_t virtual_force_input_dim = 3;
	inline constexpr std::size_t virtual_moment_input_dim = 3;
	inline constexpr std::size_t virtual_input_dim = virtual_force_input_dim + virtual_moment_input_dim;
	inline constexpr std::size_t virtual_state_input_dim = state_dim + virtual_input_dim;

	// Domain-specific
	// phi, theta, r are the integrated states
	inline constexpr std::size_t integrated_state_dim = 3;

	// The guidance setpoint vector stores psi, which is not a state variable, hence the +1
	inline constexpr std::size_t guidance_state_dim = state_dim + 1;

	inline constexpr std::size_t trim_residual_dim = 14;

	// Shorthands
	inline constexpr std::size_t nx = state_dim;
	inline constexpr std::size_t nu = input_dim;
	inline constexpr std::size_t ns = full_surface_input_dim;
	inline constexpr std::size_t np = full_propulsor_input_dim;
	inline constexpr std::size_t nxu = state_input_dim;
	inline constexpr std::size_t nv = virtual_input_dim;
	inline constexpr std::size_t nxv = virtual_state_input_dim;
	inline constexpr std::size_t nxi = integrated_state_dim;
	inline constexpr std::size_t nxg = guidance_state_dim;
	inline constexpr std::size_t nr = trim_residual_dim;

} // namespace constants
