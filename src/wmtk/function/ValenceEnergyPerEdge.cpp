#include "ValenceEnergyPerEdge.hpp"
#include <wmtk/Primitive.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
namespace wmtk::function {
ValenceEnergyPerEdge::ValenceEnergyPerEdge(const TriMesh& mesh)
    : PerSimplexFunction(mesh, PrimitiveType::Edge)
{}

double ValenceEnergyPerEdge::get_value(const Tuple& tuple) const
{
    // assume tuple is not a boundary edge
    const Tuple& current_v = tuple;
    const Tuple other_v = tri_mesh().switch_vertex(current_v);
    long val0 = static_cast<long>(SimplicialComplex::vertex_one_ring(tri_mesh(), current_v).size());
    long val1 = static_cast<long>(SimplicialComplex::vertex_one_ring(tri_mesh(), other_v).size());
    if (tri_mesh().is_boundary_vertex(current_v)) {
        val0 += 2;
    }
    if (tri_mesh().is_boundary_vertex(other_v)) {
        val1 += 2;
    }
    if (val0 < 4 || val1 < 4) {
        return -1;
    }

    /*            top_v
    //           /  \
    //          /    \
    //  current_v-----other_v
    //          \    /
    //           \  /
    //          bottom_v
    */
    const Tuple top_v = tri_mesh().switch_vertex(tri_mesh().switch_edge(current_v));
    const Tuple bottom_v =
        tri_mesh().switch_vertex(tri_mesh().switch_edge(tri_mesh().switch_face(current_v)));
    long val2 = static_cast<long>(SimplicialComplex::vertex_one_ring(tri_mesh(), top_v).size());
    long val3 = static_cast<long>(SimplicialComplex::vertex_one_ring(tri_mesh(), bottom_v).size());

    if (tri_mesh().is_boundary_vertex(top_v)) {
        val2 += 2;
    }
    if (tri_mesh().is_boundary_vertex(bottom_v)) {
        val3 += 2;
    }
    // formula from: https://github.com/daniel-zint/hpmeshgen/blob/cdfb9163ed92523fcf41a127c8173097e935c0a3/src/HPMeshGen2/TriRemeshing.cpp#L315
    const long val_energy = std::max(std::abs(val0 - 6), std::abs(val1 - 6)) +
                            std::max(std::abs(val2 - 6), std::abs(val3 - 6));
    // const long val_after = std::max(std::abs(val0 - 7), std::abs(val1 - 7)) +
    //                        std::max(std::abs(val2 - 5), std::abs(val3 - 5));

    return static_cast<double>(val_energy);
}

const TriMesh& ValenceEnergyPerEdge::tri_mesh() const
{
    return static_cast<const TriMesh&>(PerSimplexFunction::mesh());
}


} // namespace wmtk::function
