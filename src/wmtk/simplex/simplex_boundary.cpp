#include "simplex_boundary.hpp"
#include "faces.hpp"

namespace wmtk::simplex {

SimplexCollection
simplex_boundary(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    return faces(mesh, simplex, sort_and_clean);
}

} // namespace wmtk::simplex
