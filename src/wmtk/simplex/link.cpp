#include "link.hpp"

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include "closed_star.hpp"
#include "faces.hpp"
#include "link_single_dimension.hpp"
#include "top_dimension_cofaces.hpp"


namespace wmtk::simplex {

namespace {

void link_vertex(
    const TriMesh& mesh,
    const simplex::Simplex& simplex,
    std::vector<Simplex>& collection)
{
    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Triangle;

    assert(mesh.is_valid(simplex.tuple()));
    const Tuple t_in = simplex.tuple();
    Tuple t = t_in;

    do {
        const Tuple t_collect = mesh.switch_tuples(t, {PV, PE});
        collection.emplace_back(simplex::Simplex(mesh, PV, t_collect));
        collection.emplace_back(simplex::Simplex(mesh, PE, t_collect));

        if (mesh.is_boundary_edge(t)) {
            break;
        }
        t = mesh.switch_tuples(t, {PF, PE});
    } while (t != t_in);


    if (t == t_in && !mesh.is_boundary_edge(t)) {
        return;
    }

    t = mesh.switch_edge(t_in);

    collection.emplace_back(simplex::Simplex(mesh, PV, mesh.switch_tuple(t, PV)));
    if (mesh.is_boundary_edge(t)) {
        return;
    }
    t = mesh.switch_tuples(t, {PF, PE});

    do {
        const Tuple t_collect = mesh.switch_tuples(t, {PV, PE});
        collection.emplace_back(simplex::Simplex(mesh, PV, t_collect));
        collection.emplace_back(simplex::Simplex(mesh, PE, t_collect));

        if (mesh.is_boundary_edge(t)) {
            break;
        }
        t = mesh.switch_tuples(t, {PF, PE});
    } while (true);
}

// void link_edge(
//     const TriMesh& mesh,
//     const simplex::Simplex& simplex,
//     std::vector<Simplex>& collection)
// {
//     constexpr PrimitiveType PV = PrimitiveType::Vertex;
//     constexpr PrimitiveType PE = PrimitiveType::Edge;
//     constexpr PrimitiveType PF = PrimitiveType::Triangle;
//     const Tuple& t = simplex.tuple();

//     collection.emplace_back(simplex::Simplex(mesh, PV, mesh.switch_tuples(t, {PE, PV})));
//     if (!mesh.is_boundary_edge(t)) {
//         collection.emplace_back(simplex::Simplex(mesh, PV, mesh.switch_tuples(t, {PF, PE, PV})));
//     }
// }

} // namespace

SimplexCollection link(const Mesh& mesh, const simplex::Simplex& simplex, const bool sort_and_clean)
{
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Triangle:
        return link(static_cast<const TriMesh&>(mesh), simplex, sort_and_clean);
    case PrimitiveType::Tetrahedron:
        return link(static_cast<const TetMesh&>(mesh), simplex, sort_and_clean);
    case PrimitiveType::Vertex:
    case PrimitiveType::Edge:
    default: return link_slow(mesh, simplex, sort_and_clean); break;
    }
}

SimplexCollection
link(const TriMesh& mesh, const simplex::Simplex& simplex, const bool sort_and_clean)
{
    // make use of the fact that the top dimension coface tuples always contain the simplex itself

    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex: {
        std::vector<Simplex> link_simplices;
        link_vertex(mesh, simplex, link_simplices);
        SimplexCollection collection(mesh, std::move(link_simplices));
        if (sort_and_clean) {
            collection.sort();
        }
        return collection;
    }
    case PrimitiveType::Edge: {
        return link_single_dimension(mesh, simplex, PrimitiveType::Vertex, sort_and_clean);
    }
    case PrimitiveType::Triangle: break;
    case PrimitiveType::Tetrahedron:
    default: log_and_throw_error("Unknown primitive type in open_star."); break;
    }

    return SimplexCollection(mesh);
}

SimplexCollection
link(const TetMesh& mesh, const simplex::Simplex& simplex, const bool sort_and_clean)
{
    // make use of the fact that the top dimension coface tuples always contain the simplex itself
    const std::vector<Tuple> cell_tuples = top_dimension_cofaces_tuples(mesh, simplex);

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Triangle;
    constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

    std::vector<Simplex> all_cofaces;
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex:
        all_cofaces.reserve(cell_tuples.size() * 7);
        for (Tuple t : cell_tuples) {
            t = mesh.switch_tuples(t, {PV, PE, PF});

            all_cofaces.emplace_back(Simplex::face(mesh, t));

            all_cofaces.emplace_back(Simplex::edge(mesh, t));
            all_cofaces.emplace_back(Simplex::vertex(mesh, t));
            t = mesh.switch_tuples(t, {PV, PE});
            all_cofaces.emplace_back(Simplex::edge(mesh, t));
            all_cofaces.emplace_back(Simplex::vertex(mesh, t));
            t = mesh.switch_tuples(t, {PV, PE});
            all_cofaces.emplace_back(Simplex::edge(mesh, t));
            all_cofaces.emplace_back(Simplex::vertex(mesh, t));
        }
        break;
    case PrimitiveType::Edge:
        all_cofaces.reserve(cell_tuples.size() * 3);
        for (Tuple t : cell_tuples) {
            t = mesh.switch_tuples(t, {PE, PV, PF, PE});

            all_cofaces.emplace_back(Simplex::edge(mesh, t));
            all_cofaces.emplace_back(Simplex::vertex(mesh, t));
            all_cofaces.emplace_back(Simplex::vertex(mesh, mesh.switch_vertex(t)));
        }
        break;
    case PrimitiveType::Triangle:
        all_cofaces.reserve(cell_tuples.size() * 7);
        for (Tuple t : cell_tuples) {
            t = mesh.switch_tuples(t, {PE, PF, PE, PV});

            all_cofaces.emplace_back(Simplex::vertex(mesh, t));
        }
        break;
    case PrimitiveType::Tetrahedron: break;
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
