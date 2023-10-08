#pragma once
#include <functional>
#include <variant>

namespace wmtk {
class Mesh;
class PointMesh;
class EdgeMesh;
class TriMesh;
class TetMesh;
} // namespace wmtk
namespace wmtk::utils {


using MeshVariantType = std::variant<
    std::reference_wrapper<Mesh>,
    // std::reference_wrapper<PointMesh>,
    // std::reference_wrapper<EdgeMesh>,
    std::reference_wrapper<TriMesh>,
    std::reference_wrapper<TetMesh>>;
using ConstMeshVariantType = std::variant<
    std::reference_wrapper<const Mesh>,
    // std::reference_wrapper<const PointMesh>,
    // std::reference_wrapper<const EdgeMesh>,
    std::reference_wrapper<const TriMesh>,
    std::reference_wrapper<const TetMesh>>;

// converts a mesh to one of the derived classes we haev available
// if the messh type is uknown the returned type is still mesh
MeshVariantType as_mesh_variant(Mesh& m);

// const version of as_mesh_variant
ConstMeshVariantType as_const_mesh_variant(const Mesh& m);


} // namespace wmtk::utils
