#include "open_star.hpp"

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include "faces.hpp"
#include "top_dimension_cofaces.hpp"

namespace wmtk::simplex {

SimplexCollection open_star(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Triangle:
        return open_star(static_cast<const TriMesh&>(mesh), simplex, sort_and_clean);
    case PrimitiveType::Tetrahedron:
        return open_star(static_cast<const TetMesh&>(mesh), simplex, sort_and_clean);
    case PrimitiveType::Vertex:
    case PrimitiveType::Edge:
    default: return open_star_slow(mesh, simplex, sort_and_clean); break;
    }
}

SimplexCollection open_star(const TriMesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    // make use of the fact that the top dimension coface tuples always contain the simplex itself
    const std::vector<Tuple> cell_tuples = top_dimension_cofaces_tuples(mesh, simplex);

    std::vector<IdSimplex> all_cofaces;
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex:
        all_cofaces.reserve(cell_tuples.size() * 3 + 1);
        for (const Tuple& t : cell_tuples) {
            all_cofaces.emplace_back(mesh.get_id_simplex(t, PrimitiveType::Triangle));
            all_cofaces.emplace_back(mesh.get_id_simplex(t, PrimitiveType::Edge));
            all_cofaces.emplace_back(mesh.get_id_simplex(mesh.switch_edge(t), PrimitiveType::Edge));
        }
        break;
    case PrimitiveType::Edge:
        all_cofaces.reserve(cell_tuples.size() + 1);
        for (const Tuple& t : cell_tuples) {
            all_cofaces.emplace_back(mesh.get_id_simplex(t, PrimitiveType::Triangle));
        }
        break;
    case PrimitiveType::Triangle: all_cofaces.reserve(1); break;
    case PrimitiveType::Tetrahedron:
    default: break;
    }
    all_cofaces.emplace_back(mesh.get_id_simplex(simplex));

    SimplexCollection collection(mesh, std::move(all_cofaces));


    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

SimplexCollection open_star(const TetMesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    // make use of the fact that the top dimension coface tuples always contain the simplex itself
    const std::vector<Tuple> cell_tuples = top_dimension_cofaces_tuples(mesh, simplex);

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Triangle;
    constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

    std::vector<IdSimplex> all_cofaces;
    switch (simplex.primitive_type()) {
    case PrimitiveType::Vertex:
        all_cofaces.reserve(cell_tuples.size() * 7 + 1);
        for (Tuple t : cell_tuples) {
            all_cofaces.emplace_back(mesh.get_id_simplex(t, PT));
            all_cofaces.emplace_back(mesh.get_id_simplex(t, PF));
            all_cofaces.emplace_back(mesh.get_id_simplex(t, PE));
            t = mesh.switch_tuples(t, {PE, PF});
            all_cofaces.emplace_back(mesh.get_id_simplex(t, PF));
            all_cofaces.emplace_back(mesh.get_id_simplex(t, PE));
            t = mesh.switch_tuples(t, {PE, PF});
            all_cofaces.emplace_back(mesh.get_id_simplex(t, PF));
            all_cofaces.emplace_back(mesh.get_id_simplex(t, PE));
        }
        break;
    case PrimitiveType::Edge:
        all_cofaces.reserve(cell_tuples.size() * 3 + 1);
        for (const Tuple& t : cell_tuples) {
            all_cofaces.emplace_back(mesh.get_id_simplex(t, PT));
            all_cofaces.emplace_back(mesh.get_id_simplex(t, PF));
            all_cofaces.emplace_back(mesh.get_id_simplex(mesh.switch_face(t), PF));
        }
        break;
    case PrimitiveType::Triangle:
        all_cofaces.reserve(3);
        assert(cell_tuples.size() <= 2);
        for (const Tuple& t : cell_tuples) {
            all_cofaces.emplace_back(mesh.get_id_simplex(t, PT));
        }
        break;
    case PrimitiveType::Tetrahedron: all_cofaces.reserve(1); break;
    default: log_and_throw_error("Unknown primitive type in open_star."); break;
    }
    all_cofaces.emplace_back(mesh.get_id_simplex(simplex));

    SimplexCollection collection(mesh, std::move(all_cofaces));


    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

SimplexCollection
open_star_slow(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    const IdSimplex id_simplex = mesh.get_id_simplex(simplex);
    SimplexCollection collection(mesh);

    collection.add(simplex);

    const SimplexCollection top_dimension_cofaces_collection =
        top_dimension_cofaces(mesh, simplex, false);

    for (const IdSimplex& coface_cell : top_dimension_cofaces_collection.simplex_vector()) {
        collection.add(coface_cell);

        const SimplexCollection cell_boundary = faces(mesh, mesh.get_simplex(coface_cell));
        for (const IdSimplex& boundary_simplex : cell_boundary.simplex_vector()) {
            const SimplexCollection bdbd = faces(mesh, mesh.get_simplex(boundary_simplex));
            if (bdbd.contains(id_simplex)) {
                collection.add(boundary_simplex);
            }
        }
    }

    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

} // namespace wmtk::simplex
