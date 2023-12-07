#include "EdgeSplit.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/invariants/ValidTupleInvariant.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

namespace wmtk::operations {

void OperationSettings<tet_mesh::EdgeSplit>::create_invariants()
{
    invariants = std::make_shared<InvariantCollection>(m_mesh);
}

namespace tet_mesh {

EdgeSplit::EdgeSplit(TetMesh& m, const Simplex& t, const OperationSettings<EdgeSplit>& settings)
    : TetMeshOperation(m)
    , TupleOperation(settings.invariants, t)
    , m_settings{settings}
{
    assert(t.primitive_type() == PrimitiveType::Edge);
}

// EdgeSplit::~EdgeSplit() = default;

bool EdgeSplit::execute()
{
    auto return_data = mesh().split_edge(input_tuple().tuple(), hash_accessor());
    m_output_tuple = return_data.m_output_tuple;

    return true;
}

std::string EdgeSplit::name() const
{
    return "tet_mesh_split_edge";
}

Tuple EdgeSplit::new_vertex() const
{
    return m_output_tuple;
}

Tuple EdgeSplit::return_tuple() const
{
    return m_output_tuple;
}

std::vector<Tuple> EdgeSplit::modified_primitives(PrimitiveType type) const
{
    Simplex v(PrimitiveType::Vertex, m_output_tuple);
    std::vector<Tuple> ret;
    if (type == PrimitiveType::Face) {
        auto sc = SimplicialComplex::open_star(mesh(), v);
        auto faces = sc.get_simplices(PrimitiveType::Face);
        for (const auto& face : faces) {
            ret.emplace_back(face.tuple());
        }
    } else if (type == PrimitiveType::Vertex) {
        auto sc = SimplicialComplex::open_star(mesh(), v);
        auto vertices = sc.get_simplices(PrimitiveType::Vertex);
        for (const auto& vertex : vertices) {
            ret.emplace_back(vertex.tuple());
        }
    }
    return ret;
}
} // namespace tet_mesh
} // namespace wmtk::operations
