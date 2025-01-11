#pragma once

#include <filesystem>

namespace wmtk::components::utils {

/**
 * @brief Wraps the path concatenation `root / path`.
 *
 * If `path` is absolute it is returned the way it is and root is ignored.
 * If `root` is a file the containing folder is considered as root path.
 *
 * Examples:
 *      root = "a/"
 *      path = "b.txt"
 *      returns: "a/b.txt"
 *
 *      root = "a/f.json"
 *      path = "o/b.txt"
 *      returns: "a/o/b.txt"
 *
 *      root = "a/"
 *      path = "/c/b.txt"
 *      returns: "/c/b.txt"
 *
 * An exception is thrown if the root path does not exist.
 *
 * @param root The folder used as root. If it is a file the parent folder is used as root.
 * @param path The path that should be made absolute.
 *
 * @returns The absolute path.
 */
std::filesystem::path resolve_path(
    const std::filesystem::path& root,
    const std::filesystem::path& path);

/**
 * @brief Call `resolve_path()` to concatenate several paths.
 *
 * The behavior resembles: `root / path_1 / path_2 / path_3 / ...`
 */
std::filesystem::path resolve_paths(
    const std::filesystem::path& root,
    const std::initializer_list<std::filesystem::path>& paths);

/**
 * @brief Wraps `resolve_path()` but returns path directly if it is empty.
 */
std::filesystem::path resolve_path_if_not_empty(
    const std::filesystem::path& root,
    const std::filesystem::path& path);

} // namespace wmtk::components::utils
