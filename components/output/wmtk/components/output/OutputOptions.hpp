#pragma once
#include <filesystem>
#include <nlohmann/json.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>


namespace wmtk::components::output {
struct OutputOptions
{
    std::filesystem::path file;

    std::string type;

    // some formats (?msh?) have a dedicated slot for positions
    std::variant<wmtk::attribute::MeshAttributeHandle, std::string> position_attribute;


    // This was intended to be implemenetd easily with default, too lazy to properly implement now without c++20
    //bool operator==(const OutputOptions& o) const;
};
} // namespace wmtk::components::output


namespace nlohmann {
template <>
struct adl_serializer<wmtk::components::output::OutputOptions>
{
    using Type = wmtk::components::output::OutputOptions;
    static void to_json(json& j, const Type& v);
    static void from_json(const json& j, Type& v);
};
} // namespace nlohmann
