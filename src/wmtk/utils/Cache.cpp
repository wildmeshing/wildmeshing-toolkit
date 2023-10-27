#include "Cache.hpp"

#include <chrono>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>

#include <filesystem>

namespace fs = std::filesystem;

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


    const long long timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::system_clock::now().time_since_epoch())
                                    .count();

    fs::path unique_dir;
    for (size_t i = 0; i < max_tries; ++i) {
        std::stringstream ss;
        ss << std::hex << timestamp << "_" << i;
        unique_dir = tmp / (prefix + ss.str());

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

std::filesystem::path Cache::path() const
{
    return m_cache_dir;
}

Cache::~Cache()
{
    fs::remove_all(m_cache_dir);
}
