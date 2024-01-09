#include <memory>

namespace wmtk {
class Mesh;
}
namespace wmtk::components::internal {
class Grid2Options;
class Grid3Options;
class TriangleFanOptions;
class DiskOptions;
namespace procedural {

// implementations lie in the options files
std::shared_ptr<Mesh> make_mesh(const DiskOptions&);
std::shared_ptr<Mesh> make_mesh(const Grid3Options&);
std::shared_ptr<Mesh> make_mesh(const Grid2Options&);
std::shared_ptr<Mesh> make_mesh(const TriangleFanOptions&);

} // namespace procedural
} // namespace wmtk::components::internal
