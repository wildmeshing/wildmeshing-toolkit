#include "EdgeValenceEnergy.hpp"
#include <wmtk/Primitive.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/link.hpp>

namespace wmtk::function {
EdgeValenceEnergy::EdgeValenceEnergy(
    const Mesh& mesh,
    const attribute::MeshAttributeHandle& variable_attribute_handle)
    : PerSimplexFunction(mesh, PrimitiveType::Vertex, variable_attribute_handle)
{}

double EdgeValenceEnergy::get_value(const simplex::Simplex& edge_simplex) const
{
    // assume tuple is not a boundary edge
    Tuple tuple = edge_simplex.tuple();
    const Tuple& current_v = tuple;
    const Tuple other_v = tri_mesh().switch_vertex(current_v);
    int64_t val0 =
        static_cast<int64_t>(simplex::link(tri_mesh(), simplex::Simplex::vertex(current_v))
                                 .simplex_vector(PrimitiveType::Vertex)
                                 .size());
    int64_t val1 = static_cast<int64_t>(simplex::link(tri_mesh(), simplex::Simplex::vertex(other_v))
                                            .simplex_vector(PrimitiveType::Vertex)
                                            .size());
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
    int64_t val2 = static_cast<int64_t>(simplex::link(tri_mesh(), simplex::Simplex::vertex(top_v))
                                            .simplex_vector(PrimitiveType::Vertex)
                                            .size());
    int64_t val3 =
        static_cast<int64_t>(simplex::link(tri_mesh(), simplex::Simplex::vertex(bottom_v))
                                 .simplex_vector(PrimitiveType::Vertex)
                                 .size());

    if (tri_mesh().is_boundary_vertex(top_v)) {
        val2 += 2;
    }
    if (tri_mesh().is_boundary_vertex(bottom_v)) {
        val3 += 2;
    }
    // formula from: https://github.com/daniel-zint/hpmeshgen/blob/cdfb9163ed92523fcf41a127c8173097e935c0a3/src/HPMeshGen2/TriRemeshing.cpp#L315
    const int64_t val_energy = std::max(std::abs(val0 - 6), std::abs(val1 - 6)) +
                               std::max(std::abs(val2 - 6), std::abs(val3 - 6));
    // const int64_t val_after = std::max(std::abs(val0 - 7), std::abs(val1 - 7)) +
    //                        std::max(std::abs(val2 - 5), std::abs(val3 - 5));

    return static_cast<double>(val_energy);
}

const TriMesh& EdgeValenceEnergy::tri_mesh() const
{
    return static_cast<const TriMesh&>(PerSimplexFunction::mesh());
}


} // namespace wmtk::function
