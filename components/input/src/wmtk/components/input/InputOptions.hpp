#pragma once


#include <filesystem>
#include <optional>
#include <vector>

namespace wmtk::components::input {

struct InputOptions
{
    std::filesystem::path file;
    std::optional<std::vector<std::vector<std::string>>> imported_attributes;
};


} // namespace wmtk::components::input
