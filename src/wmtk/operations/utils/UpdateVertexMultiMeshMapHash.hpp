#include <wmtk/Accessor.hpp>
// #include <wmtk/EdgeMesh.hpp>
// #include <wmtk/Mesh.hpp>
// #include <wmtk/PointMesh.hpp>
// #include <wmtk/SimplicialComplex.hpp>
// #include <wmtk/TetMesh.hpp>
// #include <wmtk/TriMesh.hpp>

namespace wmtk {
class Mesh;
class PointMesh;
class EdgeMesh;
class TriMesh;
class TetMesh;
class Tuple;
class SimplicialComplex;
} // namespace wmtk


namespace wmtk::operations::utils {

void update_vertex_operation_hashes(Mesh& m, const Tuple& vertex, Accessor<long>& hash_accessor);

void update_vertex_operation_multimesh_map_hash(
    Mesh& m,
    const SimplicialComplex& vertex_closed_star,
    Accessor<long>& parent_hash_accessor);

} // namespace wmtk::operations::utils
