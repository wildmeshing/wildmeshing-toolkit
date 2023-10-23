#include "as_mesh_variant.hpp"
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include "as_variant.hpp"


namespace wmtk::utils::metaprogramming {


MeshVariantType as_mesh_variant(Mesh& mesh)
{
    return as_variant<MeshVariantTraits>(mesh);
}
ConstMeshVariantType as_const_mesh_variant(const Mesh& mesh)
{
    return as_const_variant<MeshVariantTraits>(mesh);
}
} // namespace wmtk::utils::metaprogramming
