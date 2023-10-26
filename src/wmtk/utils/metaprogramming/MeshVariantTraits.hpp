#pragma once

#include "DerivedReferenceWrapperVariantTraits.hpp"


namespace wmtk {
class Mesh;
class PointMesh;
class EdgeMesh;
class TriMesh;
class TetMesh;
} // namespace wmtk
namespace wmtk::utils::metaprogramming {
using MeshVariantTraits =
    DerivedReferenceWrapperVariantTraits<Mesh, PointMesh, EdgeMesh, TriMesh, TetMesh>;
using MeshVariantType = MeshVariantTraits::ReferenceVariant;
using ConstMeshVariantType = MeshVariantTraits::ConstReferenceVariant;

template <>
size_t MeshVariantTraits::get_index(const Mesh& m);

} // namespace wmtk::utils
