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


    nlohmann::json name_spec;

    bool old_mode = false;
    bool ignore_z = false;
};


} // namespace wmtk::components::input

namespace nlohmann {
template <>
struct adl_serializer<wmtk::components::input::InputOptions>
{
    using Type = wmtk::components::input::InputOptions;
    static void to_json(json& j, const Type& v);
    static void from_json(const json& j, Type& v);
};
} // namespace nlohmann
