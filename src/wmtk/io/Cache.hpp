#pragma once

#include <filesystem>
#include <map>
#include <string_view>
#include <wmtk/Mesh.hpp>
#include "CachedMultiMesh.hpp"

namespace wmtk::io {


class Cache
{
public:
    /**
     * This class creates and maintains a cache folder for storing temporary data. The folder is
     * placed under the path given in `directory` and starts with the `prefix`. The remainder of the
     * cache name is a hex number representing a timestamp and another one representing the number
     * of tries that were necessary to create the cache folder.
     *
     * The cache folder is automatically removed in the destructor of the class
     * if delete_cache is set to be true. If the contents of the cache should
     * be copied out, one can use `export_cache()`. Once a cache is exported, it
     * can be
     * imported again using `import_cache()`.
     */
    Cache(
        const std::string& prefix,
        const std::filesystem::path directory = "",
        bool delete_cache = true);

    Cache(Cache&&);
    Cache& operator=(Cache&&);

    ~Cache();

    /**
     * @brief Create a file with the given name in the cache without overwriting any file with the
     * same name.
     *
     * The file path is stored internally and can be accessed using `get_file_path`
     *
     * @param name The file name.
     */
    const std::filesystem::path& create_unique_file(
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
    const std::filesystem::path& get_file_path(const std::string& filename);

    /**
     * @brief Get the path where the file with the given name is stored.
     *
     * If a file with this name does not exist an error is thrown.
     *
     * @return file path
     */
    std::filesystem::path get_file_path(const std::string& filename) const;

    /**
     * @brief Get the path of the cache folder.
     *
     * @return cache path
     */
    std::filesystem::path get_cache_path() const;

    /**
     * @brief Load a mesh from cache.
     *
     * @param name The name associated with the mesh
     *
     * @return shared pointer to the mesh
     */
    std::shared_ptr<Mesh> read_mesh(const std::string& name) const;

    void load_multimesh(const std::string& name) const;

    /**
     * @brief Write a mesh to cache.
     *
     * @param mesh The mesh that is written
     * @param name The name associated with the mesh
     * @param name The name associated with the mesh
     * @param multimesh_names names to absolute_multi_mesh_id
     */
    void write_mesh(
        const Mesh& m,
        const std::string& name,
        const std::map<std::string, std::vector<int64_t>>& multimesh_names = {});

    /**
     * @brief Export the cache to the given location.
     *
     * The location must be a non-existing path. Along with all files, a json file is written that
     * contains the dictionary of names used inside the program and the actual file names relative
     * to the cache folder.
     *
     * returns true if export was successful, false otherwise
     */
    bool export_cache(const std::filesystem::path& export_location);

    /**
     * @brief Import a cache from the given location.
     *
     * The location must be cache folder that was previously generated using the `export` function.
     * The cache must be empty before importing.
     *
     * returns true if import was successful, false otherwise
     */
    bool import_cache(const std::filesystem::path& import_location);

    std::vector<int64_t> absolute_multi_mesh_id(const std::string& name) const;

    /**
     * @brief Compare two caches for equality.
     *
     * Only compares meshes registered in the cache.
     */
    bool equals(const Cache& o);

    /**
     * @brief Create a unique directory in the given location.
     *
     * The directory will consist of the given prefix, a timestamp in nanoseconds convertex to hex,
     * and a counter that is increased each time the directory generation fails.
     *
     * If the given location is empty, use the system tmp directory.
     *
     * @param prefix A prefix for the directory name.
     * @param location The location where the new directory will be created.
     * @param max_tries The maximum number of tries before the function throws a runtime error.
     */
    static std::filesystem::path create_unique_directory(
        const std::string& prefix,
        const std::filesystem::path& location = "",
        size_t max_tries = 10000);


    /// Unsets the mesh held by each cached mm - useful for debugging whether cache loading works
    void flush_multimeshes();

    /**
     * @brief Get all names of the meshes stored in cache.
     */
    std::vector<std::string> mesh_names();

private:
    std::filesystem::path m_cache_dir;
    std::map<std::string, std::filesystem::path> m_file_paths; // name --> file location
    mutable std::map<std::string, CachedMultiMesh> m_multimeshes;
    bool m_delete_cache = true;

    inline static const std::string m_cache_content_name =
        "cache_contents"; // name of the json file used for import/export
};
} // namespace wmtk::io
