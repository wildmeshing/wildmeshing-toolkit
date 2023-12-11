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
    RawSimplex(const Mesh& mesh, const std::vector<Tuple>& vertices);

    RawSimplex(std::vector<long>&& vertices);

    RawSimplex(const Mesh& mesh, const Simplex& simplex);

    long dimension() const;

    bool operator==(const RawSimplex& o) const;
    bool operator<(const RawSimplex& o) const;

    /**
     * @brief Get the face opposite to the given vertex.
     * The face consists of all vertices except for the given one.
     *
     * @param excluded_id the vertex index that is not included in the returned simplex
     *
     * @return RawSimplex representing the face opposing the given vertex
     */
    RawSimplex opposite_face(const long excluded_id);

    /**
     * @brief Get the face opposite to the given vertex.
     * The face consists of all vertices except for the given one.
     *
     * @param mesh
     * @param vertex a tuple representing the vertex that is not included in the returned simplex
     *
     * @return RawSimplex representing the face opposing the given vertex
     */
    RawSimplex opposite_face(const Mesh& mesh, const Tuple& vertex);

    /**
     * @brief Get all faces of the simplex
     */
    RawSimplexCollection faces();

private:
    std::vector<long> m_vertices;
};

} // namespace wmtk::simplex
