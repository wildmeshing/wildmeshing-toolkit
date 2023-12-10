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
 */
class RawSimplex
{
public:
    RawSimplex(const Mesh& mesh, const std::vector<Tuple>& vertices);

    RawSimplex(std::vector<long>&& vertices);

    RawSimplex(const Mesh& mesh, const Simplex& simplex);

    long dimension() const;

private:
    std::vector<long> m_vertices;
};

} // namespace wmtk::simplex
