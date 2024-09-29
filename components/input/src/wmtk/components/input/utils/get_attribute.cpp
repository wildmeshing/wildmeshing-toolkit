
#include "get_attribute.hpp"
#include <wmtk/Mesh.hpp>
#include "..//MeshCollection.hpp"
#include "wmtk/utils/Logger.hpp"
namespace wmtk::components::input::utils {

wmtk::attribute::MeshAttributeHandle get_attribute(const Mesh& mesh, const nlohmann::json& js)
{
    if(js.is_string()) {
        spdlog::trace("Parsing {} as just a name", js.dump());
    }
    const std::string name = js.is_string() ? js.get<std::string>() : js["name"].get<std::string>();
    std::string type_name = js.contains("type") ? js["type"] : "";
    int simplex_dim = js.contains("simplex") ? js["simplex"].get<int>() : -1;

    auto try_types= [&](int index) {
            const PrimitiveType pt = wmtk::get_primitive_type_from_id(index);
        // search
        if (mesh.has_attribute<double>(name, pt)) {
            type_name = "double";
            return true;
        } else if (mesh.has_attribute<int64_t>(name, pt)) {
            type_name = "int";
            return true;
        } else if (mesh.has_attribute<char>(name, pt)) {
            type_name = "char";
            return true;
        } else if (mesh.has_attribute<wmtk::Rational>(name, pt)) {
            type_name = "rational";
            return true;
        }
        return false;
    };


    // if simplex dim is missing then both simplex dim and type name is populated
    if(simplex_dim == -1) {
        for(int j = 0 ; j < mesh.top_cell_dimension(); ++j) {
            if(try_types(j)) {
                simplex_dim = j;
                break;
            }

        }
        // if simplex dim was tehre but type name not populated we populate
    } else if(type_name.empty()) {
        try_types(simplex_dim);
    }
    // only other case is both are populated, which is fine

    assert(!type_name.empty());
    assert(simplex_dim != -1);

    const PrimitiveType pt = wmtk::get_primitive_type_from_id(simplex_dim);


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
