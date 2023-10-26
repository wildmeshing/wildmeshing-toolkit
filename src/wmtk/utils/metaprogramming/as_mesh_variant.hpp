#pragma once
#include <functional>
#include <variant>
#include "MeshVariantTraits.hpp"

namespace wmtk::utils::metaprogramming {



// converts a mesh to one of the derived classes we haev available
// if the messh type is uknown the returned type is still mesh
MeshVariantType as_mesh_variant(Mesh& m);

// const version of as_mesh_variant
ConstMeshVariantType as_const_mesh_variant(const Mesh& m);


} // namespace wmtk::utils
