#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk {
namespace operations::utils {

void update_vertex_operation_hashes(Mesh& m, const Tuple& vertex);
void update_vertex_operation_multimesh_map_hash(Mesh& m);

} // namespace operations::utils
} // namespace wmtk