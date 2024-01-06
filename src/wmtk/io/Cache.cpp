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
    , m_delete_cache(o.m_delete_cache)
{
    // make sure that the other cache doesn't use delete semantics anymore
    o.m_delete_cache = false;
}
Cache& Cache::operator=(Cache&& o)
{
    m_cache_dir = std::move(o.m_cache_dir);
    m_file_paths = std::move(o.m_file_paths);
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
    const fs::path tmp = location.empty() ? std::filesystem::temp_directory_path() : location;

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

const std::filesystem::path& Cache::create_unique_file(
    const std::string& filename,
    const std::string& extension,
    size_t max_tries)
{
    const std::string timestamp = number_to_hex(nanoseconds_timestamp());

    for (size_t i = 0; i < max_tries; ++i) {
        const fs::path p =
            m_cache_dir / (filename + "_" + timestamp + "_" + number_to_hex(i) + extension);

        if (fs::exists(p)) {
            continue;
        }

        // try to touch the file
        std::ofstream ofs(p);
        if (ofs.is_open()) {
            m_file_paths[filename] = p;
            ofs.close();
            return m_file_paths[filename];
        }
        ofs.close();
    }

    throw std::runtime_error("Could not generate a unique file.");
}

const std::filesystem::path& Cache::get_file_path(const std::string& filename)
{
    const auto it = m_file_paths.find(filename);

    if (it == m_file_paths.end()) {
        // filename does not exist yet --> create it
        return create_unique_file(filename, "");
    } else {
        return it->second;
    }
}

std::filesystem::path Cache::get_file_path(const std::string& filename) const
{
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

std::shared_ptr<Mesh> Cache::read_mesh(const std::string& name) const
{
    const fs::path p = get_file_path(name);
    return wmtk::read_mesh(p);
}

void Cache::write_mesh(const Mesh& m, const std::string& name)
{
    const auto it = m_file_paths.find(name);

    fs::path p;

    if (it == m_file_paths.end()) {
        // file does not exist yet --> create it
        p = create_unique_file(name, ".hdf5");
        m_file_paths[name] = p;
    } else {
        p = it->second;
    }

    HDF5Writer writer(p);
    m.serialize(writer);
}

bool Cache::export_cache(const std::filesystem::path& export_location)
{
    if (fs::exists(export_location)) {
        return false;
    }

    fs::path cache_content_path;

    // create a json with all cached names
    {
        nlohmann::json cache_content;
        for (const auto& [first, second] : m_file_paths) {
            cache_content[first] = fs::relative(second, m_cache_dir).string();
        }

        cache_content_path = create_unique_file(m_cache_content_name, ".json");
        std::ofstream o(cache_content_path);
        o << std::setw(4) << cache_content << std::endl;
        o.close();
    }

    // copy folder to export location
    fs::copy(m_cache_dir, export_location, fs::copy_options::recursive);

    // delete json
    fs::remove(cache_content_path);

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

    // remove current directory
    fs::remove_all(m_cache_dir);
    // copy import
    fs::copy(import_location, m_cache_dir, fs::copy_options::recursive);

    // find json
    fs::path cache_content_path;
    for (const auto& f : fs::directory_iterator(m_cache_dir)) {
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
        for (auto& [first, second] : map_paths) {
            m_file_paths[first] = m_cache_dir / second;
        }
    }

    // delete json
    fs::remove(cache_content_path);

    return true;
}

bool Cache::equals(const Cache& o)
{
    // check file names
    if (m_file_paths.size() != o.m_file_paths.size() ||
        !std::equal(
            m_file_paths.begin(),
            m_file_paths.end(),
            m_file_paths.begin(),
            [](const auto& a, const auto& b) { return a.first == b.first; })) {
        wmtk::logger().info("File name list is unequal.");
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

} // namespace wmtk::io
