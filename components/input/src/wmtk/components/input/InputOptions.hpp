#pragma once


#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>
#include <vector>

namespace wmtk::components::input {

class InputOptions
{
public:
    std::filesystem::path file;
    std::optional<std::vector<std::vector<std::string>>> imported_attributes;

    std::optional<std::filesystem::path> working_directory;


    nlohmann::json name_spec;

    bool old_mode = false;
    bool ignore_z_if_zero = false;

    bool operator==(const InputOptions& o) const;
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
