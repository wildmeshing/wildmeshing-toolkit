
#include "get_attribute.hpp"
#include <ranges>
#include <wmtk/Mesh.hpp>
#include "..//MeshCollection.hpp"
#include "../NamedMultiMesh.hpp"
#include "AttributeDescription.hpp"
#include "get_attribute.hpp"
#include "wmtk/utils/Logger.hpp"

namespace wmtk::components::multimesh::utils {


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
    if (attribute_path.empty()) {
        tmp.emplace_back("");
        tmp.emplace_back("");

    } else {
        std::string v = std::string(attribute_path);
        std::istringstream iss(v);
        std::string token;
        if (v.size() > 0 && v[0] == '/') {
            tmp.emplace_back("");
        }
        while (std::getline(iss, token, '/')) {
            if (!token.empty()) tmp.emplace_back(token);
        }
        // at most 2 tokens are allowed
        assert(tmp.size() <= 2);
        if (tmp.size() == 1) {
            tmp = {"", tmp[0]};
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
