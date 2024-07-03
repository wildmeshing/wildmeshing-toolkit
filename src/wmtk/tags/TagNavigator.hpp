#pragma once
#include <algorithm>
#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>

namespace wmtk::tags {

/// Perform navigation on a tagged submesh
class TagNavigator
{
public:
    using MeshType = wmtk::Mesh;
    using TagAccessorType = wmtk::attribute::Accessor<int64_t, MeshType, 1>;
    using TagHandleType = wmtk::attribute::TypedAttributeHandle<int64_t>;
    TagNavigator(const MeshType& mesh, const TagHandleType& tag_attr, int64_t m_value = 0);
    TagNavigator(const TagAccessorType& tag_attr, int64_t m_value = 0);

    [[nodiscard]] auto switch_tuple(const Tuple& t, PrimitiveType pt) const -> Tuple;

    /// the dimension of the tagged submesh ( 2 for a trimesh )
    [[nodiscard]] auto dimension() const -> int64_t;
    /// Dimension of the mesh that holds this (cell dimension / 3 for a tetmesh)
    [[nodiscard]] auto embedded_dimension() const -> int64_t;
    /// the dimension of the tagged submesh ( 2 for a trimesh )
    [[nodiscard]] auto top_simplex_type() const -> PrimitiveType;

    [[nodiscard]] auto mesh() const -> const MeshType&;

    [[nodiscard]] auto possible_facets(const Tuple& t, const PrimitiveType pt) const
        -> std::vector<simplex::Simplex>;
    [[nodiscard]] auto possible_cofaces(const Tuple& t, const PrimitiveType pt) const
        -> std::vector<simplex::Simplex>;

private:
    const TagAccessorType m_accessor;
    const int64_t m_value;
};

inline auto TagNavigator::switch_tuple(const Tuple& t, PrimitiveType pt) const -> Tuple
{
    if (int64_t(top_simplex_type()) < dimension()) {
        return mesh().switch_tuple(t, pt);
    } else {
    }

    return {};
}
[[nodiscard]] auto TagNavigator::mesh() const -> const Mesh&
{
    return m_accessor.mesh();
}

[[nodiscard]] auto TagNavigator::dimension() const -> int64_t
{
    return m_accessor.dimension();
}
[[nodiscard]] auto TagNavigator::embedded_dimension() const -> int64_t
{
    return mesh().top_cell_dimension();
}
[[nodiscard]] auto TagNavigator::possible_facets(const Tuple& t) const
    -> std::vector<simplex::Simplex>
{
    return possible_cofaces(t, top_simplex_type());
}
[[nodiscard]] auto TagNavigator::possible_cofaces(const Tuple& t, const PrimitiveType pt) const
    -> std::vector<simplex::Simplex>
{
    assert(pt <= top_simplex_type());
    std::vector<simplex::Simplex> tups = wmtk::simplex::cofaces_single_dimension(
                                             mesh(),
                                             wmtk::simplex::Simplex(mesh(), pt, t),
                                             top_simplex_type())
                                             .simplex_vector();

    tups.erase(
        std::remove_if(
            tups.begin(),
            tups.end(),
            [&](const simplex::Simplex& s) -> bool {
                return m_accessor.const_scalar_attribute(s) != m_value;
            }),
        tups.end());
    return tups;
}
} // namespace wmtk::tags
