#pragma once


#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>
#include <vector>

namespace wmtk::components {
    namespace utils {
    class PathResolver;
    }
}
namespace wmtk::components::input {

class InputOptions
{
public:
    InputOptions();
    ~InputOptions();
    std::filesystem::path path;
    std::optional<std::vector<std::vector<std::string>>> imported_attributes;



    // either you can have a name spec in json or you can have it in a file
    nlohmann::json name_spec;
    std::optional<std::filesystem::path> name_spec_file;

    bool validate = false;

    bool prune_unused_names = false;



    // many applications use ignore_z_if_zero and imported attribute sonly works for tets. This flag enables that
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
