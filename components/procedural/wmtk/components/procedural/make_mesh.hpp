#include <memory>

namespace wmtk {
class Mesh;
}
namespace wmtk::components::procedural {
class Grid2Options;
class Grid3Options;
class GridOptions;
class TriangleFanOptions;
class DiskOptions;

// implementations lie in the options files
std::shared_ptr<Mesh> make_mesh(const DiskOptions&);
std::shared_ptr<Mesh> make_mesh(const Grid3Options&);
std::shared_ptr<Mesh> make_mesh(const GridOptions&);
std::shared_ptr<Mesh> make_mesh(const Grid2Options&);
std::shared_ptr<Mesh> make_mesh(const TriangleFanOptions&);

} // namespace wmtk::components::procedural
