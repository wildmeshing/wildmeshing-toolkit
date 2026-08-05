#include "resolve_path.hpp"

#include <filesystem>
#include <wmtk/utils/Logger.hpp>

namespace fs = std::filesystem;

namespace wmtk::utils {

fs::path resolve_path(const fs::path& root, const fs::path& path)
{
    if (path.is_absolute()) {
        return path;
    }

    // An empty root means "resolve against the current directory". Spelling that out
    // rather than leaving it to fs::absolute() is not pedantry: fs::absolute("") is
    // not portable -- libstdc++ throws filesystem_error(EINVAL), libc++ returns the
    // current path. Components take root from json_params["input_dir"], which
    // app/main.cpp derives from the config file's parent_path(), and that is empty
    // for a bare relative config name (`wmtk_app -j config.json`). Without this,
    // every component aborts on Linux for a config invoked that way, while the same
    // command works on macOS.
    const fs::path root_abs = root.empty() ? fs::current_path() : fs::absolute(root);

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

} // namespace wmtk::utils