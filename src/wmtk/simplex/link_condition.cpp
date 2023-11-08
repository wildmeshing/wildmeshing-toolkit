#include "link_condition.hpp"
#include "link.hpp"

namespace wmtk::simplex {
bool link_condition_closed_trimesh(const TriMesh& mesh, const Tuple& edge)
{
    SimplexCollection lnk_a = link(mesh, Simplex(PrimitiveType::Vertex, edge), true); // link(a)
    SimplexCollection lnk_b = link(
        mesh,
        Simplex(PrimitiveType::Vertex, mesh.switch_tuple(edge, PrimitiveType::Vertex)),
        true); // link(b)
    SimplexCollection lnk_ab = link(mesh, Simplex(PrimitiveType::Edge, edge), true); // link(ab)

    SimplexCollection lnk_a_lnk_b_intersection = SimplexCollection::get_intersection(lnk_a, lnk_b);

    return SimplexCollection::are_simplex_collections_equal(lnk_a_lnk_b_intersection, lnk_ab);
}

bool link_condition(const EdgeMesh& mesh, const Tuple& edge)
{
    // SimplexCollection collection = link(mesh, simplex, false);
    return true;
    // return collection.simplex_vector().empty();
}

bool link_condition(const TriMesh& mesh, const Tuple& edge)
{
    // step1 check link condition for closed case
    if (!link_condition_closed_trimesh(mesh, edge)) {
        return false;
    }


    // SimplexCollection collection = link(mesh, simplex, false);
    return true;
    // return collection.simplex_vector().empty();
}

} // namespace wmtk::simplex
