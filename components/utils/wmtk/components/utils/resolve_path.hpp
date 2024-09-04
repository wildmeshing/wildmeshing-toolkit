#pragma once

#include <filesystem>

namespace wmtk::components::utils {

/**
 * @brief Compute an absolute path, assuming it was relative to another file.
 *
 * If the path is already absolute, it is returned the way it is. If it is relative, this function
 * assumes that it is relative to the input_file_path. The input_file_path may represent an input
 * folder or a file within the input folder.
 *
 * Examples:
 *      path = "b.txt"
 *      input_file_path = "a/"
 *      returns: "a/b.txt"
 *
 *      path = "o/b.txt"
 *      input_file_path = "a/f.json"
 *      returns: "a/o/b.txt"
 *
 *      path = "/c/b.txt"
 *      input_file_path = "a/"
 *      returns: "/c/b.txt"
 *
 *
 * @param path The path that should be made absolute.
 * @param input_file_path The file or folder which is the root for the relative input path.
 * @param only_if_exists Modify the path only if the modified one does not exist yet.
 *
 * @returns The absolute path.
 */
std::filesystem::path resolve_path(
    const std::filesystem::path& path,
    const std::filesystem::path& input_file_path,
    const bool only_if_not_exists = false);

} // namespace wmtk::components::utils
