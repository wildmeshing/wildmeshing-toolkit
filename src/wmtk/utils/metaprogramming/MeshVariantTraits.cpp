#include "MeshVariantTraits.hpp"
#include <wmtk/Mesh.hpp>


namespace wmtk::utils::metaprogramming {

template <>
size_t MeshVariantTraits::get_index(const Mesh& m)
{
    return m.top_cell_dimension();
}
} // namespace wmtk::utils::metaprogramming
