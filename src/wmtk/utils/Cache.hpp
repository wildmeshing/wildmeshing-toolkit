#pragma once

#include <filesystem>
#include <map>

class Cache
{
public:
    Cache(const std::string& prefix, const std::filesystem::path directory = "");

    ~Cache();

    std::filesystem::path path() const;

    /**
     * @brief Create a file with the given name in the cache without overwriting any file with the
     * same name.
     *
     * The file path is stored internally and can be accessed using `get_file_path`
     *
     * @param name The file name.
     */
    std::filesystem::path create_unique_file(
        const std::string& filename,
        const std::string& extension,
        size_t max_tries = 10000);

    /**
     *
     */
    std::filesystem::path get_file_path(const std::string& filename);


private:
    std::filesystem::path m_cache_dir;
    std::map<std::string, std::filesystem::path> m_file_paths; // name --> file location
};