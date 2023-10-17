#pragma once

#include <filesystem>
#include <map>

class Cache
{
public:
    Cache(const std::filesystem::path& directory);

private:
    // should be something like tmp/wmtk_cache/
    std::filesystem::path m_cache_dir;
    std::map<std::string, std::string> m_map_name_to_unique_file;
};