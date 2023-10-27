#include "Cache.hpp"

#include <chrono>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <wmtk/utils/Logger.hpp>

#include <filesystem>

namespace fs = std::filesystem;

long long nanoseconds_timestamp()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

std::string long_to_hex(const long long& l)
{
    std::stringstream ss;
    ss << std::hex << l;
    return ss.str();
}

/**
 * @brief Create a unique directory in the given location.
 *
 * The directory will consist of the given prefix, a timestamp in nanoseconds convertex to hex, and
 * a counter that is increased each time the directory generation fails.
 *
 * If the given location is empty, use the system tmp directory.
 *
 * @param prefix A prefix for the directory name.
 * @param location The location where the new directory will be created.
 * @param max_tries The maximum number of tries before the function throws a runtime error.
 */
std::filesystem::path create_unique_directory(
    const std::string& prefix,
    const std::filesystem::path& location = "",
    size_t max_tries = 10000)
{
    const fs::path tmp = location.empty() ? std::filesystem::temp_directory_path() : location;

    const std::string timestamp = long_to_hex(nanoseconds_timestamp());

    fs::path unique_dir;
    for (size_t i = 0; i < max_tries; ++i) {
        unique_dir = tmp / (prefix + "_" + timestamp + "_" + long_to_hex(i));

        if (std::filesystem::create_directory(unique_dir)) {
            return unique_dir;
        }
    }

    throw std::runtime_error("Could not generate a unique directory.");

    return unique_dir;
}

Cache::Cache(const std::string& prefix, const std::filesystem::path location)
    : m_cache_dir(location)
{
    m_cache_dir = create_unique_directory(prefix, location);
}

Cache::~Cache()
{
    const size_t max_tries = 1000;
    for (size_t i = 0; fs::exists(m_cache_dir) && i < max_tries; ++i) {
        fs::remove_all(m_cache_dir);
    }

    if (fs::exists(m_cache_dir)) {
        wmtk::logger().warn("Could not remove cache folder {}", fs::absolute(m_cache_dir));
    }
}

std::filesystem::path Cache::path() const
{
    return m_cache_dir;
}

std::filesystem::path Cache::create_unique_file(
    const std::string& filename,
    const std::string& extension,
    size_t max_tries)
{
    const std::string timestamp = long_to_hex(nanoseconds_timestamp());

    for (size_t i = 0; i < max_tries; ++i) {
        const fs::path p =
            m_cache_dir / (filename + "_" + timestamp + "_" + long_to_hex(i) + extension);

        if (fs::exists(p)) {
            continue;
        }

        // try to touch the file
        std::ofstream ofs(p);
        if (ofs.is_open()) {
            m_file_paths[filename] = p;
            ofs.close();
            return p;
        }
        ofs.close();
    }

    throw std::runtime_error("Could not generate a unique file.");
    return "";
}

std::filesystem::path Cache::get_file_path(const std::string& filename)
{
    const auto it = m_file_paths.find(filename);

    if (it == m_file_paths.end()) {
        // filename does not exist yet --> create it
        return create_unique_file(filename, "");
    } else {
        return it->second;
    }
}
