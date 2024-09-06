#include "resolve_path.hpp"

#include <filesystem>
#include <wmtk/utils/Logger.hpp>

namespace fs = std::filesystem;

namespace wmtk::components::utils {

fs::path resolve_path(const fs::path& root, const fs::path& path)
{
    if (path.is_absolute()) {
        return path;
    }

    const fs::path root_abs = fs::absolute(root);

    const fs::path root_dir =
        fs::exists(root_abs) && !fs::is_directory(root_abs) ? root_abs.parent_path() : root_abs;


    const fs::path resolved_path = fs::weakly_canonical(root_dir / path);

    return resolved_path;
}

fs::path resolve_paths(const fs::path& root, const std::initializer_list<fs::path>& paths)
{
    fs::path p_ret = root;

    for (const fs::path& p : paths) {
        p_ret = resolve_path(p_ret, p);
    }

    return p_ret;
}

std::filesystem::path resolve_path_if_not_empty(
    const std::filesystem::path& root,
    const std::filesystem::path& path)
{
    if (path.empty()) {
        return path;
    }

    return resolve_path(path, root);
}

} // namespace wmtk::components::utils