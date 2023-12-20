#include "EdgeCollapse.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/TetMesh.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>
#include <wmtk/operations/utils/multi_mesh_edge_collapse.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>

namespace wmtk::operations {

void OperationSettings<tet_mesh::EdgeCollapse>::create_invariants()
{
    invariants = std::make_shared<InvariantCollection>(m_mesh);
    invariants->add(std::make_shared<MultiMeshLinkConditionInvariant>(m_mesh));
}

namespace tet_mesh {

EdgeCollapse::EdgeCollapse(
    TetMesh& m,
    const Simplex& t,
    const OperationSettings<EdgeCollapse>& settings)
    : TetMeshOperation(m)
    , TupleOperation(settings.invariants, t)
{
    assert(t.primitive_type() == PrimitiveType::Edge);
}

bool EdgeCollapse::execute()
{
    auto return_data = mesh().collapse_edge(input_tuple(), hash_accessor());
    m_output_tuple = return_data.m_output_tuple;
    m_deleted_tet_ids = return_data.simplex_ids_to_delete[3];

    return true;
}

std::vector<Simplex> EdgeCollapse::modified_primitives() const
{
    return {Simplex::vertex(m_output_tuple)};
}

std::vector<Simplex> EdgeCollapse::unmodified_primitives() const
{
    const simplex::Simplex v0 = simplex::Simplex::vertex(input_tuple());
    const simplex::Simplex v1 = mesh().parent_scope(
        [&]() { return simplex::Simplex::vertex(mesh().switch_vertex(input_tuple())); });
    return {v0, v1};
}

std::string EdgeCollapse::name() const
{
    return "tet_mesh_collapse_edge";
}

Tuple EdgeCollapse::return_tuple() const
{
    return m_output_tuple;
}

std::vector<long> EdgeCollapse::deleted_tet_ids() const
{
    return m_deleted_tet_ids;
}

std::vector<Tuple> EdgeCollapse::modified_tetrahedra() const
{
    Simplex v(PrimitiveType::Vertex, m_output_tuple);
    return simplex::top_dimension_cofaces_tuples(mesh(), v);
    // auto sc = simplex::top_dimension_cofaces(mesh(), v);
    // std::vector<Tuple> ret;
    // for (const auto& tet : sc) {
    //     ret.emplace_back(tet.tuple());
    // }
    // return ret;
}
} // namespace tet_mesh
} // namespace wmtk::operations
