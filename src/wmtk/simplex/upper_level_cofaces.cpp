#include "upper_level_cofaces.hpp"
#include <queue>
#include <set>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleCellLessThanFunctor.hpp>
#include "link.hpp"
#include "top_level_cofaces.hpp"
namespace wmtk::simplex {

std::vector<Tuple> upper_level_cofaces_tuples(
    const Mesh& mesh,
    const Simplex& simplex,
    const PrimitiveType& cofaces_type)
{
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Face:
        return upper_level_cofaces_tuples(static_cast<const TriMesh&>(mesh), simplex, cofaces_type);
    case PrimitiveType::Tetrahedron:
        return upper_level_cofaces_tuples(static_cast<const TetMesh&>(mesh), simplex, cofaces_type);
    default: assert(false); throw "unknown mesh type in upper_level_cofaces_tuples";
    }
}

std::vector<Tuple> upper_level_cofaces_tuples(
    const TriMesh& mesh,
    const Simplex& my_simplex,
    const PrimitiveType& cofaces_type)
{
    assert(my_simplex.primitive_type() < cofaces_type);
    std::vector<Tuple> collection;
    if (my_simplex.primitive_type() == PrimitiveType::Vertex &&
        (cofaces_type == PrimitiveType::Edge)) {
        auto sc = link(mesh, my_simplex);
        std::vector<Tuple> coface_edge_tuples;
        for (const Simplex& edge : sc.simplex_vector(PrimitiveType::Edge)) {
            coface_edge_tuples.emplace_back(mesh.switch_vertex(mesh.switch_edge(edge.tuple())));
            coface_edge_tuples.emplace_back(
                mesh.switch_vertex(mesh.switch_edge(mesh.switch_vertex(edge.tuple()))));
        }
        SimplexCollection ec(
            mesh,
            utils::tuple_vector_to_homogeneous_simplex_vector(
                coface_edge_tuples,
                PrimitiveType::Edge));
        ec.sort_and_clean();
        collection = ec.tuple_vector();
    } else {
        collection = top_level_cofaces_tuples(mesh, my_simplex);
    }
    return collection;
}
} // namespace wmtk::simplex