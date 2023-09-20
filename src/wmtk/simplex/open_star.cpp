#include "simplex_boundary.hpp"

#include "internal/OpenStar.hpp"

namespace wmtk::simplex {

SimplexCollection open_star(const Mesh& mesh, const Simplex& simplex)
{
    return internal::OpenStar(mesh, simplex);
}

} // namespace wmtk::simplex