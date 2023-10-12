#include "MeshVariantTraits.hpp"
#include <wmtk/Mesh.hpp>


namespace wmtk::utils::metaprogramming {

template <>
size_t TestRefType::get_index(const Mesh& m)
{}
} // namespace wmtk::utils::metaprogramming
