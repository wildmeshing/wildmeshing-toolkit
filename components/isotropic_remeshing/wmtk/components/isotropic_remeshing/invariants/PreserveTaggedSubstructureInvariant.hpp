#pragma once

#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/invariants/Invariant.hpp>

namespace wmtk::components::isotropic_remeshing::invariants {

class PreserveTaggedSubstructureInvariant: public wmtk::invariants::Invariant
{
public:
    PreserveTaggedSubstructureInvariant(
        const Mesh& m,
        const TypedAttributeHandle<int64_t>& substructure_tag_handle,
        const int64_t substructure_tag_value);
    bool before(const simplex::Simplex& input_simplex) const final;


    bool check_vertex(const simplex::Simplex& input_simplex) const;
    bool check_edge(const simplex::Simplex& input_simplex) const;
    bool check_tri(const simplex::Simplex& input_simplex) const;
    bool check_tet(const simplex::Simplex& input_simplex) const;

private:
    TypedAttributeHandle<int64_t> m_substructure_tag_handle;
    int64_t m_substructure_tag_value;
};

} // namespace wmtk::invariants
