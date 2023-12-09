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
    auto return_data = mesh().split_edge(input_tuple(), hash_accessor());
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

std::vector<Simplex> EdgeSplit::modified_primitives() const
{
    std::vector<Simplex> s;
    s.reserve(3);
    s.emplace_back(simplex::Simplex::vertex(new_vertex()));

    for (const auto& et : new_spine_edges()) {
        s.emplace_back(simplex::Simplex::edge(et));
    }
    return s;
}

std::array<Tuple, 2> EdgeSplit::new_spine_edges() const
{
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Face;
    constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;
    std::array<Tuple, 2> r{
        {new_vertex(), mesh().switch_tuples(new_vertex(), {PE, PF, PT, PF, PE})}};
    return r;
}
} // namespace tet_mesh
} // namespace wmtk::operations
