#include <memory>

namespace wmtk {
class Mesh;
class TriMesh;
class TetMesh;
}
namespace wmtk::components::internal {
class Grid2Options;
class Grid3Options;
class GridOptions;
class TriangleFanOptions;
class DiskOptions;
namespace procedural {

// implementations lie in the options files
std::shared_ptr<TriMesh> make_mesh(const DiskOptions&);
std::shared_ptr<TriMesh> make_mesh(const Grid2Options&);
std::shared_ptr<TriMesh> make_mesh(const TriangleFanOptions&);

std::shared_ptr<TetMesh> make_mesh(const Grid3Options&);

// Will generate a trimesh or tetmesh depending on which type of gridoptions it is passed.
// More of a helper for parsing json  to grid options where the dimension is not predetermined
std::shared_ptr<Mesh> make_mesh(const GridOptions&);
} // namespace procedural
} // namespace wmtk::components::internal
