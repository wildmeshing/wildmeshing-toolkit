#pragma once


#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>
#include <vector>

namespace wmtk::components::input {

struct InputOptions
{
    std::filesystem::path file;
    std::optional<std::vector<std::vector<std::string>>> imported_attributes;


    std::string root_name;
    nlohmann::json name_spec;

    bool old_mode = false;
    bool ignore_z = false;
};


} // namespace wmtk::components::input
