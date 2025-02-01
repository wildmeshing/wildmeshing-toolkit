
#include "get_attribute.hpp"
#include <ranges>
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/primitive_range.hpp>
#include "..//MeshCollection.hpp"
#include "../NamedMultiMesh.hpp"
#include "AttributeDescription.hpp"
#include "detail/attribute_ambiguous_error.hpp"
#include "detail/attribute_missing_error.hpp"
#include "detail/named_error_text.hpp"
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
    if (mesh.has_attribute<T>(std::string(name), pt)) {
        return mesh.get_attribute_handle<T>(std::string(name), pt);
    } else {
        throw attribute_missing_error::make(
            std::string(name),
            wmtk::get_primitive_type_id(pt),
            wmtk::attribute ::attribute_type_enum_from_type<T>());
    }
}

wmtk::attribute::MeshAttributeHandle get_attribute(
    const Mesh& mesh,
    const std::string_view& name,
    PrimitiveType pt,
    wmtk::attribute::AttributeType type)
{
    using AT = wmtk::attribute::AttributeType;
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
wmtk::attribute::MeshAttributeHandle get_attribute(
    const Mesh& mesh,
    const std::string_view& name,
    std::optional<PrimitiveType> pt,
    std::optional<wmtk::attribute::AttributeType> type)
{
    using AT = wmtk::attribute::AttributeType;
    // This order matches wmtk::components::utils::get_attributes
    constexpr static std::array<AT, 4> types{{AT::Char, AT::Int64, AT::Double, AT::Rational}};
    // constexpr static std::array<AT, 4> types{{AT::Int64, AT::Double, AT::Char, AT::Rational}};
    std::vector<AttributeDescription> possibilities;
    wmtk::attribute::MeshAttributeHandle ret;
    auto add_option = [&](PrimitiveType prim, AT t) {
        ret = get_attribute(mesh, name, prim, t);

        uint8_t dimension = wmtk::get_primitive_type_id(prim);
        possibilities.emplace_back(AttributeDescription{name, dimension, t});
    };
    if (pt.has_value() && type.has_value()) {
        add_option(pt.value(), type.value());

    } else if (pt.has_value()) {
        for (AT at : types) {
            try {
                add_option(pt.value(), at);
            } catch (const attribute_missing_error& e) {
                continue;
            }
        }
    } else if (type.has_value()) {
        for (PrimitiveType p : wmtk::utils::primitive_below(mesh.top_simplex_type())) {
            try {
                add_option(p, type.value());
            } catch (const attribute_missing_error& e) {
                continue;
            }
        }
    } else {
        for (AT at : types) {
            for (PrimitiveType p : wmtk::utils::primitive_below(mesh.top_simplex_type())) {
                try {
                    add_option(p, at);
                } catch (const attribute_missing_error& e) {
                    continue;
                }
            }
        }
    }


    if (possibilities.empty()) {
        throw attribute_missing_error::make(name, pt, type);
    } else if (possibilities.size() > 1) {
        throw attribute_ambiguous_error::make(possibilities, name,pt,type);
    }

    assert(ret.is_valid());
    return ret;
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
