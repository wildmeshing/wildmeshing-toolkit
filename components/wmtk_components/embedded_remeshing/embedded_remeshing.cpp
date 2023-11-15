#include "embedded_remeshing.hpp"

#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>


#include <wmtk/utils/Logger.hpp>

#include <wmtk/utils/mesh_utils.hpp>
#include "internal/EmbeddedRemeshing.hpp"
#include "internal/EmbeddedRemeshingOptions.hpp"

namespace wmtk {
namespace components {
// compute the length relative to the bounding box diagonal
double relative_to_absolute_length(const TriMesh& mesh, const double length_rel)
{
    auto pos_handle = mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    auto pos = mesh.create_const_accessor(pos_handle);

    Eigen::Vector3d p_max;
    p_max.setConstant(std::numeric_limits<double>::lowest());
    Eigen::Vector3d p_min;
    p_max.setConstant(std::numeric_limits<double>::max());

    for (const Tuple& v : mesh.get_all(PrimitiveType::Vertex)) {
        const Eigen::Vector3d p = pos.const_vector_attribute(v);
        p_max[0] = std::max(p_max[0], p[0]);
        p_max[1] = std::max(p_max[1], p[1]);
        p_max[2] = std::max(p_max[2], p[2]);
        p_min[0] = std::min(p_min[0], p[0]);
        p_min[1] = std::min(p_min[1], p[1]);
        p_min[2] = std::min(p_min[2], p[2]);
    }

    const double diag_length = (p_max - p_min).norm();

    return length_rel / diag_length;
}

void embedded_remeshing(std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;
    // read the image
    Eigen::MatrixXi labels_matrix(100, 100);
    int grid_x = labels_matrix.cols();
    int grid_y = labels_matrix.rows();
    Eigen::MatrixXd V(grid_x * grid_y, 3);
    for (int i = 0; i < grid_x + 1; ++i) {
        for (int j = 0; j < grid_y + 1; ++j) {
            V.row(i) << i, j, 0;
        }
    }
    RowVectors3l tris;
    tris.resize(2 * grid_x * grid_y, 3);
    for (int i = 0; i < grid_y; ++i) {
        for (int j = 0; j < grid_x; ++j) {
            int id0, id1, id2, id3;
            // 0       1
            // *-------*
            // | \___ 1|
            // | 0   \_|
            // *-------*
            // 2       3
            id0 = i * (grid_y + 1) + j;
            id1 = id0 + 1;
            id2 = id0 + grid_y + 1;
            id3 = id2 + 1;
            tris.row(id0 * 2) << id0, id2, id3;
            tris.row(id0 * 2 + 1) << id0, id3, id1;
        }
    }
    TriMesh mesh;
    mesh.initialize(tris);
    mesh_utils::set_matrix_attribute(V, "position", PrimitiveType::Vertex, mesh);
}
} // namespace components
} // namespace wmtk
