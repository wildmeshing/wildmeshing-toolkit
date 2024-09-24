#include "InputOptions.hpp"


namespace wmtk::components::input {

bool InputOptions::operator==(const InputOptions& o) const = default;
}

namespace nlohmann {
void adl_serializer<wmtk::components::input::InputOptions>::to_json(json& j, const Type& v)
{
    //
    j["file"] = v.file.string();
    if (!v.name_spec.is_null()) {
        j["name_spec"] = v.name_spec;
    }
    if (v.old_mode) {
        j["old_mode"] = true;
        j["ignore_z"] = v.ignore_z;
        if (v.imported_attributes.has_value()) {
            assert(v.imported_attributes->size() > 3);
            j["tetrahedron_attributes"] = v.imported_attributes.value()[3];
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
        v.file = j["file"].get<std::string>();
        return;
    }
    v.file = j["file"].get<std::string>();
    if (j.contains("name_spec")) {
        v.name_spec = j["name_spec"];
    }
    if (j.contains("old_mode") && bool(j["old_mode"])) {
        v.old_mode = true;
        v.ignore_z = j.contains("ignore_z") ? bool(j["ignore_z"]) : false;
        if (j.contains("tetrahedron_attributes")) {
            v.imported_attributes = {
                {},
                {},
                {},
                j["tetrahedron_attributes"].get<std::vector<std::string>>()};
        }
    } else {
        if (v.imported_attributes.has_value()) {
            v.imported_attributes = j["imported_attributes"];
        }
    }
}
} // namespace nlohmann
