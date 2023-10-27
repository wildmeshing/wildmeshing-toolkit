#pragma once

#include <filesystem>
#include <map>

class Cache
{
public:
    /**
     * This class creates and maintains a cache folder for storing temporary data. The folder is
     * placed under the path given in `directory` and starts with the `prefix`. The remainder of the
     * cache name is a hex number representing a timestamp and another one representing the number
     * of tries that were necessary to create the cache folder.
     *
     * The cache folder is automatically removed in the destructor of the class. If the contents of
     * the cache should be stored, one can use the `export()` function. Once a cache is exported, it
     * can be imported again using `import()`.
     */
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
     * @brief Get the path where the file with the given name is stored.
     *
     * If a file with this name does not exist yet, it is created using `create_unique_file`.
     *
     * @return file path
     */
    std::filesystem::path get_file_path(const std::string& filename);

    /**
     * @brief Export the cache to the given location.
     *
     * The location must be a non-existing path. Along with all files, a json file is written that
     * contains the dictionary from names, used inside the program, and actual file names relative
     * to the cache folder.
     *
     * returns true if export was successful, false otherwise
     */
    bool export(const std::filesystem::path& export_location);

    /**
     * @brief Import a cache from the given location.
     *
     * The location must be cache folder that was previously generated using the `export` function.
     * The cache must be empty before importing.
     *
     * returns true if import was successful, false otherwise
     */
    bool import(const std::filesystem::path& import_location);


private:
    std::filesystem::path m_cache_dir;
    std::map<std::string, std::filesystem::path> m_file_paths; // name --> file location
};