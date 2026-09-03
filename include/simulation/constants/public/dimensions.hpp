#pragma once
#include <cstddef>

namespace constants {

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

} // namespace constants
