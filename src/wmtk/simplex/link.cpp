#include "link.hpp"

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include "closed_star.hpp"
#include "faces.hpp"
#include "top_dimension_cofaces.hpp"

namespace wmtk::simplex {

SimplexCollection link(const Mesh& mesh, const simplex::Simplex& simplex, const bool sort_and_clean)
{
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Face:
        return link(static_cast<const TriMesh&>(mesh), simplex, sort_and_clean);
    case PrimitiveType::Tetrahedron:
        return link(static_cast<const TetMesh&>(mesh), simplex, sort_and_clean);
    case PrimitiveType::Vertex:
    case PrimitiveType::Edge:
    case PrimitiveType::HalfEdge:
    default: return link_slow(mesh, simplex, sort_and_clean); break;
    }
}

SimplexCollection
link(const TriMesh& mesh, const simplex::Simplex& simplex, const bool sort_and_clean)
{
    // make use of the fact that the top dimension coface tuples always contain the simplex itself
    const std::vector<Tuple> cell_tuples = top_dimension_cofaces_tuples(mesh, simplex);

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;

    std::vector<Simplex> all_cofaces;
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex:
        all_cofaces.reserve(cell_tuples.size() * 7);
        for (Tuple t : cell_tuples) {
            t = mesh.switch_tuples(t, {PV, PE});
            all_cofaces.emplace_back(Simplex::edge(t));
            all_cofaces.emplace_back(Simplex::vertex(t));
            t = mesh.switch_tuples(t, {PV});
            all_cofaces.emplace_back(Simplex::vertex(t));
        }
        break;
    case PrimitiveType::Edge:
        all_cofaces.reserve(cell_tuples.size() * 3);
        for (Tuple t : cell_tuples) {
            t = mesh.switch_tuples(t, {PE, PV});
            all_cofaces.emplace_back(Simplex::vertex(t));
        }
        break;
    case PrimitiveType::Face: break;
    case PrimitiveType::Tetrahedron:
    case PrimitiveType::HalfEdge:
    default: log_and_throw_error("Unknown primitive type in open_star."); break;
    }

    SimplexCollection collection(mesh, std::move(all_cofaces));

    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

SimplexCollection
link(const TetMesh& mesh, const simplex::Simplex& simplex, const bool sort_and_clean)
{
    // make use of the fact that the top dimension coface tuples always contain the simplex itself
    const std::vector<Tuple> cell_tuples = top_dimension_cofaces_tuples(mesh, simplex);

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Face;
    constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

    std::vector<Simplex> all_cofaces;
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex:
        all_cofaces.reserve(cell_tuples.size() * 7);
        for (Tuple t : cell_tuples) {
            t = mesh.switch_tuples(t, {PV, PE, PF});

            all_cofaces.emplace_back(Simplex::face(t));

            all_cofaces.emplace_back(Simplex::edge(t));
            all_cofaces.emplace_back(Simplex::vertex(t));
            t = mesh.switch_tuples(t, {PV, PE});
            all_cofaces.emplace_back(Simplex::edge(t));
            all_cofaces.emplace_back(Simplex::vertex(t));
            t = mesh.switch_tuples(t, {PV, PE});
            all_cofaces.emplace_back(Simplex::edge(t));
            all_cofaces.emplace_back(Simplex::vertex(t));
        }
        break;
    case PrimitiveType::Edge:
        all_cofaces.reserve(cell_tuples.size() * 3);
        for (Tuple t : cell_tuples) {
            t = mesh.switch_tuples(t, {PE, PV, PF, PE});

            all_cofaces.emplace_back(Simplex::edge(t));
            all_cofaces.emplace_back(Simplex::vertex(t));
            all_cofaces.emplace_back(Simplex::vertex(mesh.switch_vertex(t)));
        }
        break;
    case PrimitiveType::Face:
        all_cofaces.reserve(cell_tuples.size() * 7);
        for (Tuple t : cell_tuples) {
            t = mesh.switch_tuples(t, {PE, PF, PE, PV});

            all_cofaces.emplace_back(Simplex::vertex(t));
        }
        break;
    case PrimitiveType::Tetrahedron: break;
    case PrimitiveType::HalfEdge:
    default: log_and_throw_error("Unknown primitive type in open_star."); break;
    }

    SimplexCollection collection(mesh, std::move(all_cofaces));

    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

SimplexCollection
link_slow(const Mesh& mesh, const simplex::Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(mesh);

    SimplexCollection cs = closed_star(mesh, simplex, sort_and_clean);

    SimplexCollection simplex_w_bd = faces(mesh, simplex, false);
    simplex_w_bd.add(simplex);
    simplex_w_bd.sort_and_clean();

    for (const Simplex& s : cs.simplex_vector()) {
        SimplexCollection bd = faces(mesh, s, false);
        bd.add(s);
        bd.sort_and_clean();
        SimplexCollection intersection = SimplexCollection::get_intersection(simplex_w_bd, bd);
        if (intersection.simplex_vector().empty()) {
            collection.add(s);
        }
    }

    return collection;
}

} // namespace wmtk::simplex
