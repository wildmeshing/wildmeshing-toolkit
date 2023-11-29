#include "EdgeCollapse.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

namespace wmtk::operations {

OperationSettings<tet_mesh::EdgeCollapse>::OperationSettings() {}

void OperationSettings<tet_mesh::EdgeCollapse>::initialize_invariants(const TetMesh& m)
{
    // outdated + is valid tuple
    invariants = basic_invariant_collection(m);
}

bool OperationSettings<tet_mesh::EdgeCollapse>::are_invariants_initialized() const
{
    return find_invariants_in_collection_by_type<ValidTupleInvariant>(invariants);
}

namespace tet_mesh {

EdgeCollapse::EdgeCollapse(
    TetMesh& m,
    const Tuple& t,
    const OperationSettings<EdgeCollapse>& settings)
    : TetMeshOperation(m)
    , TupleOperation(settings.invariants, t)
{
    assert(settings.are_invariants_initialized());
}

bool EdgeCollapse::execute()
{
    auto return_data = mesh().collapse_edge(input_tuple(), hash_accessor());
    m_output_tuple = return_data.m_output_tuple;

    return true;
}

std::vector<Tuple> EdgeCollapse::modified_primitives(PrimitiveType type) const
{
    if (type == PrimitiveType::Face) {
        return modified_triangles();
    } else {
        return {};
    }
}

std::string EdgeCollapse::name() const
{
    return "tet_mesh_collapse_edge";
}

Tuple EdgeCollapse::return_tuple() const
{
    return m_output_tuple;
}

std::vector<Tuple> EdgeCollapse::modified_triangles() const
{
    Simplex v(PrimitiveType::Vertex, m_output_tuple);
    auto sc = SimplicialComplex::open_star(mesh(), v);
    auto faces = sc.get_simplices(PrimitiveType::Face);
    std::vector<Tuple> ret;
    for (const auto& face : faces) {
        ret.emplace_back(face.tuple());
    }
    return ret;
}
} // namespace tet_mesh
} // namespace wmtk::operations
