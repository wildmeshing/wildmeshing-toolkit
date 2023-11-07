#include "link_condition.hpp"
#include "link.hpp"

namespace wmtk::simplex {
bool link_condition_closed_trimesh(const TriMesh& mesh, const Tuple& edge)
{
    SimplexCollection lnk_a = link(mesh, Simplex(PrimitiveType::Vertex, edge), false); // link(a)
    SimplexCollection lnk_b = link(
        mesh,
        Simplex(PrimitiveType::Vertex, mesh.switch_tuple(edge, PrimitiveType::Vertex)),
        false); // link(b)
    return true;
    // return collection.simplex_vector().empty();
}

bool link_condition(const EdgeMesh& mesh, const Tuple& edge)
{
    // SimplexCollection collection = link(mesh, simplex, false);
    return true;
    // return collection.simplex_vector().empty();
}

bool link_condition(const TriMesh& mesh, const Tuple& edge)
{
    // SimplexCollection collection = link(mesh, simplex, false);
    return true;
    // return collection.simplex_vector().empty();
}

} // namespace wmtk::simplex
