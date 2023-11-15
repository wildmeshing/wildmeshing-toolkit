#include "TopDimensionCofacesIterable.hpp"

#include <wmtk/simplex/top_dimension_cofaces.hpp>

namespace wmtk::simplex {


TopDimensionCofacesIterable::TopDimensionCofacesIterable(const Mesh& mesh, const Simplex& simplex)
    : m_collection(top_dimension_cofaces(mesh, simplex))
{}

} // namespace wmtk::simplex
