#include "EdgeSwapValence.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"
namespace wmtk::operations::tri_mesh {
EdgeSwapValence::EdgeSwapValence(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<EdgeSwapValence>& settings)
    : EdgeSwapBase(m, t, settings.base_settings)
//, m_settings{settings}
{}

std::string EdgeSwapValence::name() const
{
    return "tri_mesh_edge_swap_valence";
}

bool EdgeSwapValence::execute()
{
    const simplex::Simplex f0 = simplex::Simplex::face(input_tuple());
    const simplex::Simplex f1 = simplex::Simplex::face(mesh().switch_face(input_tuple()));
    const std::vector<Tuple> vertices_t0 =
        simplex::faces_single_dimension_tuples(mesh(), f0, PrimitiveType::Vertex);
    const std::vector<Tuple> vertices_t1 =
        simplex::faces_single_dimension_tuples(mesh(), f1, PrimitiveType::Vertex);
    const Tuple v0 = vertices_t0[0];
    const Tuple v1 = vertices_t0[1];
    const Tuple v2 = vertices_t0[2];
    const Tuple v3 = vertices_t1[2];
    long val0 = static_cast<long>(SimplicialComplex::vertex_one_ring(mesh(), v0).size());
    long val1 = static_cast<long>(SimplicialComplex::vertex_one_ring(mesh(), v1).size());
    long val2 = static_cast<long>(SimplicialComplex::vertex_one_ring(mesh(), v2).size());
    long val3 = static_cast<long>(SimplicialComplex::vertex_one_ring(mesh(), v3).size());
    if (mesh().is_boundary_vertex(v0)) {
        val0 += 2;
    }
    if (mesh().is_boundary_vertex(v1)) {
        val1 += 2;
    }
    if (mesh().is_boundary_vertex(v2)) {
        val2 += 2;
    }
    if (mesh().is_boundary_vertex(v3)) {
        val3 += 2;
    }

    // formula from: https://github.com/daniel-zint/hpmeshgen/blob/cdfb9163ed92523fcf41a127c8173097e935c0a3/src/HPMeshGen2/TriRemeshing.cpp#L315
    const long val_before = std::max(std::abs(val0 - 6), std::abs(val1 - 6)) +
                            std::max(std::abs(val2 - 6), std::abs(val3 - 6));
    const long val_after = std::max(std::abs(val0 - 7), std::abs(val1 - 7)) +
                           std::max(std::abs(val2 - 5), std::abs(val3 - 5));

    if (val_after >= val_before) {
        return false;
    }

    if (!tri_mesh::EdgeSwapBase::execute()) {
        return false;
    }

    return true;
}


} // namespace wmtk::operations::tri_mesh
