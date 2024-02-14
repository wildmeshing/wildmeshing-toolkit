#include "link_single_dimension.hpp"

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include "closed_star.hpp"
#include "faces.hpp"
#include "top_dimension_cofaces.hpp"

namespace wmtk::simplex {

SimplexCollection link_single_dimension(
    const Mesh& mesh,
    const simplex::Simplex& simplex,
    const PrimitiveType link_type,
    const bool sort_and_clean)
{
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Triangle:
        return link_single_dimension(
            static_cast<const TriMesh&>(mesh),
            simplex,
            link_type,
            sort_and_clean);
    case PrimitiveType::Tetrahedron:
        return link_single_dimension(
            static_cast<const TetMesh&>(mesh),
            simplex,
            link_type,
            sort_and_clean);
    case PrimitiveType::Vertex:
    case PrimitiveType::Edge:
    default: return link_single_dimension_slow(mesh, simplex, link_type, sort_and_clean); break;
    }
}

SimplexCollection link_single_dimension(
    const TriMesh& mesh,
    const simplex::Simplex& simplex,
    const PrimitiveType link_type,
    const bool sort_and_clean)
{
    // make use of the fact that the top dimension coface tuples always contain the simplex itself
    const std::vector<Tuple> cell_tuples = top_dimension_cofaces_tuples(mesh, simplex);

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;

    std::vector<Simplex> all_cofaces;
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex:
        if (link_type == PrimitiveType::Vertex) {
            all_cofaces.reserve(cell_tuples.size() * 2);
            for (Tuple t : cell_tuples) {
                t = mesh.switch_tuples(t, {PV, PE});
                all_cofaces.emplace_back(Simplex::vertex(t));
                t = mesh.switch_tuples(t, {PV});
                all_cofaces.emplace_back(Simplex::vertex(t));
            }
        } else if (link_type == PrimitiveType::Edge) {
            all_cofaces.reserve(cell_tuples.size());
            for (Tuple t : cell_tuples) {
                t = mesh.switch_tuples(t, {PV, PE});
                all_cofaces.emplace_back(Simplex::edge(t));
            }
        }
        break;
    case PrimitiveType::Edge:
        if (link_type == PrimitiveType::Vertex) {
            all_cofaces.reserve(cell_tuples.size());
            for (Tuple t : cell_tuples) {
                t = mesh.switch_tuples(t, {PE, PV});
                all_cofaces.emplace_back(Simplex::vertex(t));
            }
        }
        break;
    case PrimitiveType::Triangle: break;
    case PrimitiveType::Tetrahedron:
    default: log_and_throw_error("Unknown primitive type in link_single_dimension."); break;
    }

    SimplexCollection collection(mesh, std::move(all_cofaces));

    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

SimplexCollection link_single_dimension(
    const TetMesh& mesh,
    const simplex::Simplex& simplex,
    const PrimitiveType link_type,
    const bool sort_and_clean)
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
        if (link_type == PrimitiveType::Vertex) {
            all_cofaces.reserve(cell_tuples.size() * 3);
            for (Tuple t : cell_tuples) {
                t = mesh.switch_tuples(t, {PV, PE, PF});
                all_cofaces.emplace_back(Simplex::vertex(t));
                t = mesh.switch_tuples(t, {PV, PE});
                all_cofaces.emplace_back(Simplex::vertex(t));
                t = mesh.switch_tuples(t, {PV, PE});
                all_cofaces.emplace_back(Simplex::vertex(t));
            }
        } else if (link_type == PrimitiveType::Edge) {
            all_cofaces.reserve(cell_tuples.size() * 3);
            for (Tuple t : cell_tuples) {
                t = mesh.switch_tuples(t, {PV, PE, PF});
                all_cofaces.emplace_back(Simplex::edge(t));
                t = mesh.switch_tuples(t, {PV, PE});
                all_cofaces.emplace_back(Simplex::edge(t));
                t = mesh.switch_tuples(t, {PV, PE});
                all_cofaces.emplace_back(Simplex::edge(t));
            }
        }
        if (link_type == PrimitiveType::Triangle) {
            all_cofaces.reserve(cell_tuples.size());
            for (Tuple t : cell_tuples) {
                t = mesh.switch_tuples(t, {PV, PE, PF});
                all_cofaces.emplace_back(Simplex::face(t));
            }
        }
        break;
    case PrimitiveType::Edge:
        if (link_type == PrimitiveType::Vertex) {
            all_cofaces.reserve(cell_tuples.size() * 2);
            for (Tuple t : cell_tuples) {
                t = mesh.switch_tuples(t, {PE, PV, PF, PE});
                all_cofaces.emplace_back(Simplex::vertex(t));
                all_cofaces.emplace_back(Simplex::vertex(mesh.switch_vertex(t)));
            }
        }
        if (link_type == PrimitiveType::Edge) {
            all_cofaces.reserve(cell_tuples.size());
            for (Tuple t : cell_tuples) {
                t = mesh.switch_tuples(t, {PE, PV, PF, PE});
                all_cofaces.emplace_back(Simplex::edge(t));
            }
        }
        break;
    case PrimitiveType::Triangle:
        if (link_type == PrimitiveType::Vertex) {
            all_cofaces.reserve(cell_tuples.size());
            for (Tuple t : cell_tuples) {
                t = mesh.switch_tuples(t, {PE, PF, PE, PV});
                all_cofaces.emplace_back(Simplex::vertex(t));
            }
        }
        break;
    case PrimitiveType::Tetrahedron: break;
    default: log_and_throw_error("Unknown primitive type in link_single_dimension."); break;
    }

    SimplexCollection collection(mesh, std::move(all_cofaces));

    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

SimplexCollection link_single_dimension_slow(
    const Mesh& mesh,
    const simplex::Simplex& simplex,
    const PrimitiveType link_type,
    const bool sort_and_clean)
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

    return SimplexCollection(mesh, collection.simplex_vector(link_type));
}

} // namespace wmtk::simplex
