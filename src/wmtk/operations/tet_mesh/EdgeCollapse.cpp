#include "EdgeCollapse.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

namespace wmtk::operations {

void OperationSettings<tet_mesh::EdgeCollapse>::create_invariants()
{
    invariants = std::make_shared<InvariantCollection>(m_mesh);
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

    return true;
}

std::vector<Simplex> EdgeCollapse::modified_primitives() const
{
    return {Simplex::vertex(m_output_tuple)};
}

std::vector<Simplex> EdgeCollapse::unmodified_primitives() const
{
    throw std::runtime_error("not implemented");
    return {Simplex::vertex(m_output_tuple)};
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
