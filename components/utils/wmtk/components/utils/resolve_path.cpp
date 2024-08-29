#include "resolve_path.hpp"


#include <filesystem>

namespace wmtk::components::base {
std::string
resolve_path(const std::string& path, const std::string& input_file_path, const bool only_if_exists)
{
    if (path.empty()) {
        return path;
    }

    std::filesystem::path resolved_path(path);
    if (resolved_path.is_absolute()) {
        return resolved_path.string();
    } else if (std::filesystem::exists(resolved_path)) {
        return std::filesystem::weakly_canonical(resolved_path).string();
    }

    std::filesystem::path input_dir_path(input_file_path);
    if (!std::filesystem::is_directory(input_dir_path))
        input_dir_path = input_dir_path.parent_path();

    resolved_path = std::filesystem::weakly_canonical(input_dir_path / resolved_path);

    if (only_if_exists && !std::filesystem::exists(resolved_path)) {
        return path; // return path unchanged
    }
    return resolved_path.string();
}

} // namespace wmtk::components::base