#include "PreserveTaggedSubstructureInvariant.hpp"
#include <wmtk/Mesh.hpp>

#include <wmtk/simplex/RawSimplexCollection.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/simplex/open_star.hpp>
#include "wmtk/simplex/IdSimplex.hpp"

namespace wmtk::components::isotropic_remeshing::invariants {
PreserveTaggedSubstructureInvariant::PreserveTaggedSubstructureInvariant(
    const Mesh& m,
    const TypedAttributeHandle<int64_t>& substructure_tag_handle,
    const int64_t substructure_tag_value)
    : Invariant(m, true, false, false)
    , m_substructure_tag_handle(substructure_tag_handle)
    , m_substructure_tag_value(substructure_tag_value)
{}

bool PreserveTaggedSubstructureInvariant::check_vertex(const simplex::Simplex& input_simplex) const
{
    const auto tag_acc = mesh().create_const_accessor<int64_t, 1>(m_substructure_tag_handle);
    if (tag_acc.const_scalar_attribute(input_simplex) == m_substructure_tag_value) {
        return true;
    } else if (
        tag_acc.const_scalar_attribute(
            mesh().switch_tuple(input_simplex.tuple(), PrimitiveType::Vertex)) ==
        m_substructure_tag_value) {
        return true;
    }
    return false;
}
bool PreserveTaggedSubstructureInvariant::check_edge(const simplex::Simplex& input_simplex) const
{
    const auto tag_acc = mesh().create_const_accessor<int64_t, 1>(m_substructure_tag_handle);
    simplex::IdSimplex input_edge = mesh().get_id_simplex(input_simplex);
    return tag_acc.const_scalar_attribute(input_simplex) == m_substructure_tag_value;
}
bool PreserveTaggedSubstructureInvariant::check_tri(const simplex::Simplex& input_simplex) const
{
    return true;
}
bool PreserveTaggedSubstructureInvariant::check_tet(const simplex::Simplex& input_simplex) const
{
    return true;
}

bool PreserveTaggedSubstructureInvariant::before(const simplex::Simplex& input_simplex) const
{
    assert(input_simplex.primitive_type() == PrimitiveType::Edge);

    switch (m_substructure_tag_handle.primitive_type())

    {
    case PrimitiveType::Vertex: return check_vertex(input_simplex);
    case PrimitiveType::Edge: return check_edge(input_simplex);
    case PrimitiveType::Triangle: return check_tri(input_simplex);
    case PrimitiveType::Tetrahedron: return check_tet(input_simplex);
    default: break;
    }
    return true;
}

} // namespace wmtk::components::isotropic_remeshing::invariants
