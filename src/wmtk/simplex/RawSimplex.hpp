#pragma once

#include <array>
#include <vector>
#include <wmtk/Mesh.hpp>
#include <wmtk/Tuple.hpp>

namespace wmtk::simplex {
class Simplex;
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

private:
    std::vector<long> m_vertices;
};

} // namespace wmtk::simplex
