#include "get_attributes.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/primitive_range.hpp>

namespace wmtk::components::base {

namespace {


attribute::MeshAttributeHandle get_attribute(const Mesh& m, const std::string& name)
{
    int64_t count = 0;

    std::vector<attribute::MeshAttributeHandle> handles;

    for (const PrimitiveType& ptype : utils::primitive_below(m.top_simplex_type())) {
        if (m.has_attribute<char>(name, ptype)) {
            handles.emplace_back(m.get_attribute_handle<char>(name, ptype));
        }
        if (m.has_attribute<int64_t>(name, ptype)) {
            handles.emplace_back(m.get_attribute_handle<int64_t>(name, ptype));
        }
        if (m.has_attribute<double>(name, ptype)) {
            handles.emplace_back(m.get_attribute_handle<double>(name, ptype));
        }
        if (m.has_attribute<Rational>(name, ptype)) {
            handles.emplace_back(m.get_attribute_handle<Rational>(name, ptype));
        }
    }

    if (handles.empty()) {
        log_and_throw_error("Attribute with name {} was not found.", name);
    }
    if (handles.size() > 1) {
        log_and_throw_error(
            "Attribute name ambiguity: {} attributes have the name {}",
            handles.size(),
            name);
    }


    return handles.front();
}

attribute::MeshAttributeHandle
get_attribute(const io::Cache& cache, const Mesh& m, const nlohmann::json& attribute)
{
    if (attribute.is_string()) {
        // TODO: is this appropriate? the mesh passed in should probably be ignored
        return get_attribute(m, attribute.get<std::string>());
    } else if (attribute.is_object()) {
        const std::string name = attribute["name"];
        const std::string mesh = attribute["mesh"];

        const auto& child_ptr = cache.read_mesh(mesh);

        return get_attribute(*child_ptr, name);
    }

    log_and_throw_error("Invalid type for {}.", attribute);
}

} // namespace


std::vector<attribute::MeshAttributeHandle>
get_attributes(const io::Cache& cache, const Mesh& m, const nlohmann::json& attribute_names)
{
    std::vector<attribute::MeshAttributeHandle> handles;

    if (attribute_names.is_array()) {
        for (const auto& name : attribute_names) {
            handles.push_back(get_attribute(cache, m, name));
        }
    } else {
        handles.push_back(get_attribute(cache, m, attribute_names));
    }

    return handles;
}

} // namespace wmtk::components::base
