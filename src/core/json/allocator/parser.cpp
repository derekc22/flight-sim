#include <nlohmann/json.hpp>
#include "core/json/allocator/private/parsing.hpp"
#include "core/json/allocator/public/parser.hpp"
#include "core/json/public/files.hpp"
#include "simulation/allocator/public/manager.hpp"

namespace json {

    allocator::AllocatorManager parse_allocator_config() {
        const auto config_path = resolve_run_config_entry_path("allocator_config");
        const auto config = read_json_file(config_path);
        return parse_allocator_manager(config);
    }
}
