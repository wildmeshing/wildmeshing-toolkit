#include "ExtremeOptSplit.hpp"
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {


void OperationSettings<tri_mesh::ExtremeOptSplit>::initialize_invariants(const TriMesh& m)
{
    split_settings.initialize_invariants(m);
    split_settings.invariants.add(
        std::make_shared<MinEdgeLengthInvariant>(m, position, min_squared_length));
}

bool OperationSettings<tri_mesh::ExtremeOptSplit>::are_invariants_initialized() const
{
    return split_settings.are_invariants_initialized() &&
           find_invariants_in_collection_by_type<MinEdgeLengthInvariant>(split_settings.invariants);
}
namespace tri_mesh {
ExtremeOptSplit::ExtremeOptSplit(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<ExtremeOptSplit>& settings)
    : EdgeSplit(m, t, settings.split_settings)
    , m_mesh(dynamic_cast<TriMesh&>(m))
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_uv_accessor{settings.uv_mesh_ptr->create_accessor(settings.uv_handle)}
    , m_settings{settings}
{
    coord0 = m_pos_accessor.vector_attribute(input_tuple());
    coord1 = m_pos_accessor.vector_attribute(mesh().switch_vertex(input_tuple()));

    const auto input_tuples_uv =
        m_mesh.map_to_child_tuples(*m_settings.uv_mesh_ptr, Simplex::edge(input_tuple()));

    for (const auto& input_tuple_uv : input_tuples_uv) {
        coord0s_uv.push_back(m_uv_accessor.vector_attribute(input_tuple_uv));
        coord1s_uv.push_back(
            m_uv_accessor.vector_attribute(m_settings.uv_mesh_ptr->switch_vertex(input_tuple_uv)));
    }
}
std::string ExtremeOptSplit::name() const
{
    return "tri_mesh_split_edge_at_midpoint_extreme_opt";
}
Tuple ExtremeOptSplit::return_tuple() const
{
    return m_output_tuple;
}
bool ExtremeOptSplit::before() const
{
    return TupleOperation::before();
}
bool ExtremeOptSplit::execute()
{
    if (!EdgeSplit::execute()) {
        return false;
    }
    m_output_tuple = EdgeSplit::return_tuple();
    m_pos_accessor.vector_attribute(m_output_tuple) = 0.5 * (coord0 + coord1);

    const auto output_tuples_uv =
        m_mesh.map_to_child_tuples(*m_settings.uv_mesh_ptr, Simplex::vertex(m_output_tuple));

    assert(output_tuples_uv.size() == coord0s_uv.size());

    for (size_t i = 0; i < output_tuples_uv.size(); ++i) {
        m_uv_accessor.vector_attribute(output_tuples_uv[i]) = 0.5 * (coord0s_uv[i] + coord1s_uv[i]);
    }

    return true;
}
} // namespace tri_mesh
} // namespace wmtk::operations
