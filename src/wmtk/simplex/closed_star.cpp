#include "closed_star.hpp"

#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include "faces.hpp"
#include "top_dimension_cofaces.hpp"

namespace wmtk::simplex {

SimplexCollection closed_star(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    assert(mesh.is_valid_slow(simplex.tuple()));
    SimplexCollection collection = top_dimension_cofaces(mesh, simplex, false);

    const size_t n_top_dimension_cofaces = collection.simplex_vector().size();

    // reserve memory for all cells, their faces, and the input simplex
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Vertex: break;
    case PrimitiveType::Edge: collection.reserve(n_top_dimension_cofaces * 3 + 1); break;
    case PrimitiveType::Face: collection.reserve(n_top_dimension_cofaces * 7 + 1); break;
    case PrimitiveType::Tetrahedron: collection.reserve(n_top_dimension_cofaces * 14 + 1); break;
    case PrimitiveType::HalfEdge:
    default: log_and_throw_error("unknown mesh type in top_dimension_cofaces_tuples");
    }

    for (size_t i = 0; i < n_top_dimension_cofaces; ++i) {
        faces(collection, collection.simplex_vector()[i], false);
    }

    collection.add(simplex);

    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

} // namespace wmtk::simplex
