#pragma once
#include <filesystem>
#include <memory>
#include <nlohmann/json_fwd.hpp>
#include <vector>
#include "json_macros.hpp"

namespace wmtk::components::utils {


// a more portable abstraction of the resolve_path class
class PathResolver
{
public:
    PathResolver();
    ~PathResolver();
    PathResolver(PathResolver&&);
    PathResolver(const PathResolver&);
    PathResolver& operator=(PathResolver&&);
    PathResolver& operator=(const PathResolver&);

    void add_path(const std::filesystem::path& path);

    // always returns a weakly_canonical upon success
    std::pair<std::filesystem::path, bool> resolve(const std::filesystem::path& path);

    std::vector<std::filesystem::path> get_paths() const;

    // always returns a weakly_canonical path upon success
    static std::pair<std::filesystem::path, bool> try_resolving_path(
        const std::filesystem::path& potential_base,
        const std::filesystem::path& path);

    // json expects either a single string or an array of strings
    WMTK_NLOHMANN_JSON_FRIEND_DECLARATION(PathResolver)

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
} // namespace wmtk::components::utils
