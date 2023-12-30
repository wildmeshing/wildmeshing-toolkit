#pragma once

#include <wmtk/Accessor.hpp>
// #include <wmtk/EdgeMesh.hpp>
// #include <wmtk/Mesh.hpp>
// #include <wmtk/PointMesh.hpp>
// #include <wmtk/TetMesh.hpp>
// #include <wmtk/TriMesh.hpp>

namespace wmtk {
class Mesh;
class PointMesh;
class EdgeMesh;
class TriMesh;
class TetMesh;
class Tuple;
namespace simplex {
class SimplexCollection;
}
} // namespace wmtk


namespace wmtk::operations::utils {

void update_vertex_operation_hashes(Mesh& m, const Tuple& vertex, Accessor<int64_t>& hash_accessor);

void update_vertex_operation_multimesh_map_hash(
    Mesh& m,
    const simplex::SimplexCollection& vertex_closed_star,
    Accessor<int64_t>& parent_hash_accessor);

} // namespace wmtk::operations::utils
