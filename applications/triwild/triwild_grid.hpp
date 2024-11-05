#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::triwild {

wmtk::TriMesh generate_bg_grid(
    const double x_min,
    const double y_min,
    const double x_max,
    const double y_max,
    const double length_rel,
    const double margin_eps = 0.1);

} // namespace wmtk::triwild