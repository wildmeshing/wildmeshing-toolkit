#include "TriMeshValenceFunction.hpp"
#include <wmtk/Primitive.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/utils/Logger.hpp>
namespace wmtk {
namespace function {
TriMeshValenceFunction::TriMeshValenceFunction(const TriMesh& mesh)
    : Function(mesh)
{}

double TriMeshValenceFunction::get_value(const Tuple& tuple) const
{
    // assume tuple is not a boundary edge
    const Tuple current_v = tuple;
    const Tuple other_v = m_mesh.switch_vertex(tuple);
    long val0 = static_cast<long>(SimplicialComplex::vertex_one_ring(m_mesh, current_v).size());
    long val1 = static_cast<long>(SimplicialComplex::vertex_one_ring(m_mesh, other_v).size());
    if (m_mesh.is_boundary_vertex(current_v)) {
        val0 += 2;
    }
    if (m_mesh.is_boundary_vertex(other_v)) {
        val1 += 2;
    }
    if (val0 < 4 || val1 < 4) {
        return -1;
    }

    //            top_v
    //           /  \
    //          /    \
    //  current_v-----other_v
    //          \    /
    //           \  /
    //          bottom_v
    const Tuple top_v = m_mesh.switch_vertex(m_mesh.switch_edge(tuple));
    const Tuple bottom_v = m_mesh.switch_vertex(m_mesh.switch_edge(m_mesh.switch_face(tuple)));
    long val2 = static_cast<long>(SimplicialComplex::vertex_one_ring(m_mesh, top_v).size());
    long val3 = static_cast<long>(SimplicialComplex::vertex_one_ring(m_mesh, bottom_v).size());

    if (m_mesh.is_boundary_vertex(top_v)) {
        val2 += 2;
    }
    if (m_mesh.is_boundary_vertex(bottom_v)) {
        val3 += 2;
    }
    // formula from: https://github.com/daniel-zint/hpmeshgen/blob/cdfb9163ed92523fcf41a127c8173097e935c0a3/src/HPMeshGen2/TriRemeshing.cpp#L315
    const long val_energy = std::max(std::abs(val0 - 6), std::abs(val1 - 6)) +
                            std::max(std::abs(val2 - 6), std::abs(val3 - 6));
    // const long val_after = std::max(std::abs(val0 - 7), std::abs(val1 - 7)) +
    //                        std::max(std::abs(val2 - 5), std::abs(val3 - 5));

    return static_cast<double>(val_energy);
}
} // namespace function
} // namespace wmtk