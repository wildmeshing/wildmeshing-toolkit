#include "InputOptions.hpp"
#include <wmtk/components/utils/PathResolver.hpp>
#include <wmtk/components/utils/json_utils.hpp>
#include <wmtk/utils/Logger.hpp>


namespace wmtk::components::input {

InputOptions::InputOptions() = default;
InputOptions::~InputOptions() = default;
bool InputOptions::operator==(const InputOptions& o) const = default;

} // namespace wmtk::components::input

namespace nlohmann {
void adl_serializer<wmtk::components::input::InputOptions>::to_json(json& j, const Type& v)
{
    //
    j["file"] = v.file;
    j["file"] = v.file.string();
    if (!v.name_spec.is_null()) {
        assert(!v.name_spec_file.has_value());
        j["name_spec"] = v.name_spec;
    } else if (v.name_spec_file.has_value()) {
        j["name_spec_file"] = v.name_spec_file.value();
    }

    if (v.old_mode) {
        j["old_mode"] = true;
        j["ignore_z"] = v.ignore_z_if_zero; // keep around for deprecation purposes
        // j["ignore_z_if_zero"] = v.ignore_z_if_zero;
        if (v.imported_attributes.has_value()) {
            const auto& imported_attrs = v.imported_attributes.value();
            if (imported_attrs.size() > 3) {
                j["tetrahedron_attributes"] = imported_attrs[3];
            }
        }
    } else {
        if (v.imported_attributes.has_value()) {
            j["imported_attributes"] = v.imported_attributes.value();
        }
    }
}
void adl_serializer<wmtk::components::input::InputOptions>::from_json(const json& j, Type& v)
{
    if (j.is_string()) {
        v.file = j.get<std::filesystem::path>();
        return;
    }
    v.file = j["file"].get<std::filesystem::path>();
    if (j.contains("name_spec")) {
        v.name_spec = j["name_spec"];
    }
    if (j.contains("name_spec_file")) {
        v.name_spec_file = j["name_spec_file"].get<std::filesystem::path>();
    }


    v.old_mode = false;
    if (j.contains("old_mode")) {
        v.old_mode = bool(j["old_mode"]);
    } else {
        v.old_mode = j.contains("ignore_z") || j.contains("tetrahedron_attributes");
        if (v.old_mode) {
            wmtk::logger().debug(
                "Input component is using old mode because ignore_z exists ({}) or "
                "tetrahedron_attributes exists ({})",
                j.contains("ignore_z"),
                j.contains("tetrahedron_attributes"));
        }
    }


    if (v.old_mode) {
        v.ignore_z_if_zero = j.contains("ignore_z") ? bool(j["ignore_z"]) : false;
        // overwrite old ignore_z
        // v.ignore_z_if_zero = j.contains("ignore_z_if_zero") ? bool(j["ignore_z_if_zero"]) :
        // false;
        if (j.contains("tetrahedron_attributes")) {
            v.imported_attributes = {
                {},
                {},
                {},
                j["tetrahedron_attributes"].get<std::vector<std::string>>()};
        }
    } else {
        if (j.contains("imported_attributes")) {
            v.imported_attributes = j["imported_attributes"].get<std::vector<std::vector<std::string>>>();
        }
    }
}

} // namespace nlohmann
