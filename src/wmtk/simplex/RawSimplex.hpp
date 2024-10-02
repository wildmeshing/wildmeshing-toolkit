#pragma once

#include <array>
#include <vector>
#include <wmtk/Mesh.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk::simplex {
class Simplex;
class RawSimplexCollection;

/**
 * A meshless implementation of the simplex that just stores an array of ids.
 * It is used for scenarios where a mesh does not exist.
 *
 * It must not be used for "degenerated" simplices that contain the same vertex several times, e.g.
 * {0,1,2,0} because it would be reduced to {0,1,2}.
 */
class RawSimplex
{
public:
    RawSimplex();

    RawSimplex(const Mesh& mesh, const std::vector<Tuple>& vertices);

    RawSimplex(std::vector<int64_t>&& vertices);

    RawSimplex(const Mesh& mesh, const Simplex& simplex);

    int64_t dimension() const;

    bool operator==(const RawSimplex& o) const;
    bool operator<(const RawSimplex& o) const;

    /**
     * @brief Get the face opposite to the given vertex.
     * The face consists of all vertices except for the given one.
     *
     * @param excluded_id The vertex index that is not included in the returned simplex.
     *
     * @return RawSimplex representing the face opposing the given vertex.
     */
    RawSimplex opposite_face(const int64_t excluded_id);

    /**
     * @brief Get the face opposite to the given vertex.
     * The face consists of all vertices except for the given one.
     *
     * @param mesh
     * @param vertex A tuple representing the vertex that is not included in the returned simplex.
     *
     * @return RawSimplex representing the face opposing the given vertex.
     */
    RawSimplex opposite_face(const Mesh& mesh, const Tuple& vertex);

    /**
     * @brief Get the face opposite to the given face.
     * The opposite face consists of all vertices except for the one that belong to the given face.
     *
     * @param face A RawSimplex representing that part of the simplex that is not included in the
     * returned simplex.
     *
     * @return RawSimplex representing the face opposing the given face.
     */
    RawSimplex opposite_face(const RawSimplex& face);

    /**
     * @brief Get all faces of the simplex
     */
    RawSimplexCollection faces();

private:
    std::vector<int64_t> m_vertices;
};

} // namespace wmtk::simplex
