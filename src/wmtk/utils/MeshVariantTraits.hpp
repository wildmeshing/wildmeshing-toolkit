#pragma once

#include <wmtk/utils/metaprogramming/DerivedReferenceWrapperVariantTraits.hpp>


namespace wmtk {
class Mesh;
class PointMesh;
class EdgeMesh;
class TriMesh;
class TetMesh;
} // namespace wmtk
namespace wmtk::utils {
using MeshVariantTraits = metaprogramming::
    DerivedReferenceWrapperVariantTraits<Mesh, PointMesh, EdgeMesh, TriMesh, TetMesh>;

namespace metaprogramming {
template <>
size_t TestRefType::get_index(const Mesh& m);

}
} // namespace wmtk::utils
