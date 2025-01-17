#pragma once
#include <filesystem>
#include <nlohmann/json.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/components/utils/json_macros.hpp>


namespace wmtk::components::output {
struct OutputOptions
{
    std::filesystem::path path;

    std::string type;

    // some formats (?msh?) have a dedicated slot for positions
    std::variant<wmtk::attribute::MeshAttributeHandle, std::string> position_attribute;

    // mesh name info will be serialized to a json file if available
    std::optional<std::filesystem::path> mesh_name_path;

    // This was intended to be implemenetd easily with default, too lazy to properly implement now
    // without c++20
    // bool operator==(const OutputOptions& o) const;


    bool operator<=>(const OutputOptions& o) const = default;
    WMTK_NLOHMANN_JSON_FRIEND_DECLARATION(OutputOptions)
};
} // namespace wmtk::components::output

