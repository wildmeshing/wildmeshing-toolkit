
#include "get_attribute.hpp"
#include <wmtk/Mesh.hpp>
#include "..//MeshCollection.hpp"
#include "wmtk/utils/Logger.hpp"
namespace wmtk::components::input::utils {

wmtk::attribute::MeshAttributeHandle get_attribute(const Mesh& mesh, const nlohmann::json& js)
{
    const std::string name = js["name"];
    std::string type_name = js.contains("type") ? js["type"] : "";
    const int simplex_dim = js.contains("simplex") ? js["simplex"].get<int>() : 0;
    const PrimitiveType pt = wmtk::get_primitive_type_from_id(simplex_dim);
    if (type_name.empty()) {
        // search
        if (mesh.has_attribute<double>(name, pt)) {
            type_name = "double";
        } else if (mesh.has_attribute<int64_t>(name, pt)) {
            type_name = "int";
        } else if (mesh.has_attribute<char>(name, pt)) {
            type_name = "char";
        } else if (mesh.has_attribute<wmtk::Rational>(name, pt)) {
            type_name = "rational";
        }
    }


    if (type_name == "double") {
        using T = double;
        return mesh.get_attribute_handle<T>(name, pt);
    } else if (type_name == "int") {
        using T = int64_t;
        return mesh.get_attribute_handle<T>(name, pt);
    } else if (type_name == "char") {
        using T = char;
        return mesh.get_attribute_handle<T>(name, pt);
    } else if (type_name == "rational") {
        using T = wmtk::Rational;
        return mesh.get_attribute_handle<T>(name, pt);
    } else {
        wmtk::log_and_throw_error(
            "get_attribute got an attribute called {} of type {}, not in int/double/char/rational",
            name,
            type_name);
        return {};
    }
}

wmtk::attribute::MeshAttributeHandle get_attribute(
    const wmtk::components::input::MeshCollection& mc,
    const nlohmann::json& js)
{
    const std::string name = js.contains("mesh") ? js["mesh"] : "";
    const auto& mesh = mc.get_mesh(name);
    return get_attribute(mesh, js);
}
} // namespace wmtk::components::input::utils
