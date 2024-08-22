#include "Cache.hpp"

#include <chrono>
#include <exception>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/Logger.hpp>

#include <filesystem>

namespace fs = std::filesystem;

int64_t nanoseconds_timestamp()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

std::string number_to_hex(int64_t l)
{
    return fmt::format("{0:x}", l);
}

namespace wmtk::io {

Cache::Cache(Cache&& o)
    : m_cache_dir(std::move(o.m_cache_dir))
    , m_file_paths(std::move(o.m_file_paths))
    , m_multimeshes(std::move(o.m_multimeshes))
    , m_delete_cache(o.m_delete_cache)
{
    // make sure that the other cache doesn't use delete semantics anymore
    o.m_delete_cache = false;
}
Cache& Cache::operator=(Cache&& o)
{
    m_cache_dir = std::move(o.m_cache_dir);
    m_file_paths = std::move(o.m_file_paths);
    m_multimeshes = std::move(o.m_multimeshes);
    m_delete_cache = o.m_delete_cache;
    // make sure that the other cache doesn't use delete semantics anymore
    o.m_delete_cache = false;
    return *this;
}

std::filesystem::path Cache::create_unique_directory(
    const std::string& prefix,
    const std::filesystem::path& location,
    size_t max_tries)
{
    const fs::path tmp = location.empty() ? std::filesystem::current_path() : location;

    const std::string timestamp = number_to_hex(nanoseconds_timestamp());

    fs::path unique_dir;
    for (size_t i = 0; i < max_tries; ++i) {
        unique_dir = tmp / (prefix + "_" + timestamp + "_" + number_to_hex(i));

        if (std::filesystem::create_directory(unique_dir)) {
            return unique_dir;
        }
    }

    throw std::runtime_error("Could not generate a unique directory.");
}

Cache::Cache(const std::string& prefix, const std::filesystem::path location, bool delete_cache)
    : m_cache_dir(location)
    , m_delete_cache(delete_cache)
{
    m_cache_dir = create_unique_directory(prefix, location);
    wmtk::logger().info(
        "Cache stores in folder: {}\nThe folder will be deleted automatically after successful "
        "completion.",
        fs::absolute(m_cache_dir));
}

Cache::Cache()
    : m_delete_cache(false)
{
    wmtk::logger().info("Cache does not write to disk.");
}

Cache::~Cache()
{
    if (m_delete_cache) {
        const size_t max_tries = 1000;
        for (size_t i = 0; fs::exists(m_cache_dir) && i < max_tries; ++i) {
            fs::remove_all(m_cache_dir);
        }

        if (fs::exists(m_cache_dir)) {
            wmtk::logger().warn("Could not remove cache folder {}", fs::absolute(m_cache_dir));
        }
    }
}

const std::filesystem::path& Cache::create_cache_file(
    const std::string& filename,
    const std::string& extension)
{
    throw_memory_only_mode();

    const fs::path p = m_cache_dir / (filename + extension);

    if (fs::exists(p)) {
        wmtk::logger().warn("File '{}' already exists and might be overwritten.");
    }

    // try to touch the file
    std::ofstream ofs(p);
    if (ofs.is_open()) {
        ofs.close();
    } else {
        log_and_throw_error("Cannot create file '{}'", p);
    }

    m_file_paths[filename] = p;
    return m_file_paths[filename];
}

const std::filesystem::path& Cache::get_file_path(const std::string& filename)
{
    throw_memory_only_mode();

    const auto it = m_file_paths.find(filename);

    if (it == m_file_paths.end()) {
        // filename does not exist yet --> create it
        return create_cache_file(filename, "");
    } else {
        return it->second;
    }
}

std::filesystem::path Cache::get_file_path(const std::string& filename) const
{
    throw_memory_only_mode();

    const auto it = m_file_paths.find(filename);

    if (it == m_file_paths.end()) {
        // filename does not exist yet --> create it
        throw std::runtime_error("File with name '" + filename + "' does not exist in cache");
    } else {
        return it->second;
    }
}

std::filesystem::path Cache::get_cache_path() const
{
    return m_cache_dir;
}
std::vector<int64_t> Cache::absolute_multi_mesh_id(const std::string& name) const
{
    auto mm_name = name.substr(0, name.find('.'));
    return m_multimeshes.at(mm_name).get_id_from_path(name);
}

void Cache::load_multimesh(const std::string& name) const
{
    auto mm_name = name.substr(0, name.find('.'));
    if (m_multimeshes.find(mm_name) == m_multimeshes.end()) {
        m_multimeshes.emplace(
            mm_name,
            CachedMultiMesh(mm_name, std::map<std::string, std::vector<int64_t>>{}));
    }
    CachedMultiMesh& cmm = m_multimeshes.at(mm_name);
    if (!bool(cmm.get_root())) {
        if (!m_cache_dir.empty()) {
            // load from file
            const fs::path p = get_file_path(mm_name);
            cmm.load(p);
        } else {
            // load from memory
            const auto it = m_meshes.find(mm_name);
            if (it == m_meshes.end()) {
                log_and_throw_error("A mesh with the name {} does not exist", mm_name);
            } else {
                // cmm.load(it->second);

                // HACK because Mesh cannot be copied
                Mesh& m = *(it->second);
                HDF5Writer writer("dummy.hdf5");
                m.serialize(writer, &m);
                cmm.load("dummy.hdf5");

                fs::remove("dummy.hdf5");
            }
        }
    }
}

std::shared_ptr<Mesh> Cache::read_mesh(const std::string& name) const
{
    auto mm_name = name.substr(0, name.find('.'));
    load_multimesh(mm_name);
    return m_multimeshes.at(mm_name).get_from_path(name);
}
void Cache::flush_multimeshes()
{
    for (auto& pr : m_multimeshes) {
        pr.second.flush();
    }
}

std::vector<std::string> Cache::mesh_names()
{
    std::vector<std::string> names;
    for (const auto& fp : m_file_paths) {
        names.emplace_back(fp.first);
    }
    for (const auto& fp : m_meshes) {
        names.emplace_back(fp.first);
    }

    return names;
}

void Cache::write_mesh(
    const Mesh& m,
    const std::string& name,
    const std::map<std::string, std::vector<int64_t>>& multimesh_names)
{
    // write to file
    if (!m_cache_dir.empty()) {
        const auto it = m_file_paths.find(name);
        fs::path p;
        if (it == m_file_paths.end()) {
            // file does not exist yet --> create it
            p = create_cache_file(name, ".hdf5");
            m_file_paths[name] = p;
        } else {
            p = it->second;
        }
        HDF5Writer writer(p);
        m.serialize(writer, &m);
    } else {
        // write in meshes list
        const auto it = m_meshes.find(name);
        std::shared_ptr<Mesh> pm = const_cast<Mesh&>(m).shared_from_this();

        if (it == m_meshes.end()) {
            m_meshes[name] = pm;
        } else {
            it->second = pm;
        }
    }

    // undocumented multimesh stuff - no clue what it does
    std::map<std::string, std::vector<int64_t>> mm_names = multimesh_names;
    if (m_multimeshes.find(name) != m_multimeshes.end()) {
        mm_names = m_multimeshes.at(name).get_multimesh_names();
        mm_names.insert(multimesh_names.begin(), multimesh_names.end());
        m_multimeshes.at(name) = CachedMultiMesh(
            name,
            mm_names,
            const_cast<Mesh&>(m).get_multi_mesh_root().shared_from_this());
    } else {
        m_multimeshes.emplace(
            name,
            CachedMultiMesh(
                name,
                multimesh_names,
                const_cast<Mesh&>(m).get_multi_mesh_root().shared_from_this()));
    }
}

bool Cache::export_cache(const std::filesystem::path& export_location)
{
    if (fs::exists(export_location)) {
        return false;
    }

    fs::create_directory(export_location);

    fs::path cache_content_path = export_location / (m_cache_content_name + ".json");

    // create a json with all cached names
    {
        nlohmann::json cache_content;
        if (!m_cache_dir.empty()) {
            for (const auto& [first, second] : m_file_paths) {
                cache_content[first] = fs::relative(second, m_cache_dir).string();
            }
        } else {
            for (const auto& [name, _] : m_meshes) {
                cache_content[name] = name + ".hdf5";
            }
        }

        std::ofstream o(cache_content_path);
        o << std::setw(4) << cache_content << std::endl;
        o.close();
    }

    // copy folder to export location
    if (!m_cache_dir.empty()) {
        fs::copy(m_cache_dir, export_location, fs::copy_options::recursive);
    } else {
        for (const auto& [name, m] : m_meshes) {
            HDF5Writer writer(export_location / (name + ".hdf5"));
            m->serialize(writer, m.get());
        }
    }

    //// delete json
    // fs::remove(cache_content_path);
    // m_file_paths.erase(m_cache_content_name);

    return true;
}

bool Cache::import_cache(const std::filesystem::path& import_location)
{
    if (!fs::exists(import_location)) {
        return false;
    }
    if (!m_file_paths.empty()) {
        return false;
    }
    if (!m_meshes.empty()) {
        return false;
    }

    if (!m_cache_dir.empty()) {
        // remove current directory
        fs::remove_all(m_cache_dir);
        // copy import
        fs::copy(import_location, m_cache_dir, fs::copy_options::recursive);
    }

    // find json
    fs::path cache_content_path;
    for (const auto& f : fs::directory_iterator(import_location)) {
        const fs::path p = f.path();
        if (p.stem().string().rfind(m_cache_content_name, 0) == 0) {
            cache_content_path = p;
            break;
        }
    }

    if (cache_content_path.empty()) {
        return false;
    }

    // read json
    {
        std::ifstream i(cache_content_path);
        const nlohmann::json cache_content = nlohmann::json::parse(i);

        std::map<std::string, std::string> map_paths =
            cache_content.get<std::map<std::string, std::string>>();

        // make file paths absolute
        if (!m_cache_dir.empty()) {
            for (auto& [first, second] : map_paths) {
                m_file_paths[first] = m_cache_dir / second;
            }
        } else {
            // load meshes
            for (const auto& [name, file] : map_paths) {
                // TODO check if file is a mesh
                m_meshes[name] = wmtk::read_mesh(import_location / file);
            }
        }
    }

    return true;
}

bool Cache::equals(const Cache& o)
{
    // check file names
    if (m_file_paths.size() != o.m_file_paths.size() ||
        !std::equal(
            m_file_paths.begin(),
            m_file_paths.end(),
            o.m_file_paths.begin(),
            [](const auto& a, const auto& b) { return a.first == b.first; })) {
        wmtk::logger().info("File name list is unequal.");
        return false;
    }

    if (m_meshes.size() != o.m_meshes.size() ||
        !std::equal(
            m_meshes.begin(),
            m_meshes.end(),
            o.m_meshes.begin(),
            [](const auto& a, const auto& b) { return a.first == b.first; })) {
        wmtk::logger().info("Mesh list is unequal.");
        return false;
    }

    // check files for equality
    for (const auto& [file_name, path1] : m_file_paths) {
        const auto& path2 = o.m_file_paths.at(file_name);

        std::shared_ptr<Mesh> mesh_ptr_1 = wmtk::read_mesh(path1);
        std::shared_ptr<Mesh> mesh_ptr_2 = wmtk::read_mesh(path2);

        if (!(*mesh_ptr_1 == *mesh_ptr_2)) {
            wmtk::logger().info("Mesh {} is unequal.", file_name);
            return false;
        }
    }

    return true;
}

void Cache::throw_memory_only_mode() const
{
    if (m_cache_dir.empty()) {
        log_and_throw_error("Cache is storing in memory only.");
    }
}

} // namespace wmtk::io
