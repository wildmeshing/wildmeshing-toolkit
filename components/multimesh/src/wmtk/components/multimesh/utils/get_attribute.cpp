
#include "get_attribute.hpp"
#include <ranges>
#include <wmtk/Mesh.hpp>
#include "..//MeshCollection.hpp"
#include "../NamedMultiMesh.hpp"
#include "AttributeDescription.hpp"
#include "get_attribute.hpp"
#include "wmtk/utils/Logger.hpp"

namespace wmtk::components::multimesh::utils {

wmtk::attribute::MeshAttributeHandle get_attribute_from_json(
    const Mesh& mesh,
    const nlohmann::json& js)
{
    if (js.is_string()) {
        spdlog::trace("Parsing {} as just a name", js.dump());
    }
    const std::string name = js.is_string() ? js.get<std::string>() : js["name"].get<std::string>();
    std::string type_name = js.contains("type") ? js["type"] : "";
    int simplex_dim = js.contains("simplex") ? js["simplex"].get<int>() : -1;

    auto try_types = [&](int index) {
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
    if (simplex_dim == -1) {
        for (int j = 0; j < mesh.top_cell_dimension(); ++j) {
            if (try_types(j)) {
                simplex_dim = j;
                break;
            }
        }
        // if simplex dim was tehre but type name not populated we populate
    } else if (type_name.empty()) {
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

wmtk::attribute::MeshAttributeHandle get_attribute_from_json(
    const wmtk::components::multimesh::MeshCollection& mc,
    const nlohmann::json& js)
{
    const std::string name = js.contains("mesh") ? js["mesh"] : "";
    const auto& mesh = mc.get_mesh(name);
    return get_attribute_from_json(mesh, js);
}

namespace detail {
// turns an attribute path mesh.path/attrname to mesh.path attrname
// std::array<std::string_view, 2>
auto decompose_attribute_path(std::string_view attribute_path)
{
#if defined(WMTK_ENABLED_CPP20)
    using namespace std;
    auto tmp = std::ranges::views::split(attribute_path, "/"sv) |
               std::views::transform([](const auto& r) noexcept -> std::string_view {
                   return std::string_view(r.begin(), r.end());
               });

    std::array<std::string_view, 2> ret;
    std::ranges::copy(tmp, ret.begin());
    if (ret[1].empty()) {
        std::swap(ret[0], ret[1]);
    }
    return ret;
#else

    std::array<std::string, 2> ret;
    std::vector<std::string> tmp;
    if (view.empty()) {
        tmp.emplace_back("");
        tmp.emplace_back("");

    } else {
        std::string v = std::string(view);
        std::istringstream iss(v);
        std::string token;
        if (v.size() > 0 && v[0] == '/') {
            r.emplace_back("");
        }
        while (std::getline(iss, token, '/')) {
            if (!token.empty()) tmp.emplace_back(token);
        }
        // at most 2 tokens are allowed
        assert(tmp.size() <= 2);
        if (tmp.size() == 1) {
            tmp = {"", r[0]};
        }
    }
    return std::array<std::string, 2>{{tmp[0], tmp[1]}};
#endif
}

// std::array<std::string_view, 2>
auto decompose_attribute_path(const AttributeDescription& description)
{
    return decompose_attribute_path(description.path);
}
template <typename T>
wmtk::attribute::MeshAttributeHandle
get_attribute(const Mesh& mesh, const std::string_view& name, PrimitiveType pt)
{
    return mesh.get_attribute_handle<T>(std::string(name), pt);
}

wmtk::attribute::MeshAttributeHandle get_attribute(
    const Mesh& mesh,
    const std::string_view& name,
    PrimitiveType pt,
    attribute::AttributeType type)
{
    using AT = attribute::AttributeType;
    switch (type) {
#define ENTRY(TYPE)                                                                   \
    case TYPE:                                                                        \
        return get_attribute<wmtk::attribute::type_from_attribute_type_enum_t<TYPE>>( \
            mesh,                                                                     \
            name,                                                                     \
            pt);
        ENTRY(AT::Char);
        ENTRY(AT::Double);
        ENTRY(AT::Int64);
        ENTRY(AT::Rational);
#undef ENTRY
    default: assert(false);
    }
    return {};
}
} // namespace detail

wmtk::attribute::MeshAttributeHandle get_attribute(
    const NamedMultiMesh& mesh,
    const AttributeDescription& description)
{
    auto [mesh_path, attribute_name] = detail::decompose_attribute_path(description);
    return detail::get_attribute(
        mesh.get_mesh(mesh_path),
        attribute_name,
        description.primitive_type(),
        description.type);
}
wmtk::attribute::MeshAttributeHandle get_attribute(
    const MeshCollection& mesh,
    const AttributeDescription& description)
{
    auto [mesh_path, attribute_name] = detail::decompose_attribute_path(description);

    const Mesh& nmm = mesh.get_mesh(mesh_path);
    return detail::get_attribute(
        nmm,
        attribute_name,
        description.primitive_type(),
        description.type);
}

wmtk::attribute::MeshAttributeHandle get_attribute(
    const Mesh& mesh,
    const AttributeDescription& description)
{
    auto [mesh_path, attribute_name] = detail::decompose_attribute_path(description);
    return detail::get_attribute(
        mesh,
        attribute_name,
        description.primitive_type(),
        description.type);
}

} // namespace wmtk::components::multimesh::utils
