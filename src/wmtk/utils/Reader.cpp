#include "Reader.hpp"
namespace wmtk {
void reader(std::string input_surface, Eigen::MatrixXd& VI, Eigen::MatrixXi& FI)
{
    GEO::initialize();
    GEO::Mesh input;
    GEO::mesh_load(input_surface, input);
    VI.resize(input.vertices.nb(), 3);
    for (int i = 0; i < VI.rows(); i++)
        VI.row(i) << (input.vertices.point(i))[0], (input.vertices.point(i))[1],
            (input.vertices.point(i))[2];
    input.facets.triangulate();
    // wmtk::logger().info("V {} F {}", input.vertices.nb(), input.facets.nb());
    FI.resize(input.facets.nb(), 3);
    for (int i = 0; i < FI.rows(); i++)
        FI.row(i) << input.facets.vertex(i, 0), input.facets.vertex(i, 1),
            input.facets.vertex(i, 2);
}
void input_formatter(
    std::vector<Eigen::Vector3d>& verts,
    std::vector<std::array<size_t, 3>>& tris,
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F)
{
    for (int i = 0; i < V.rows(); i++) {
        verts[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tris[i][j] = (size_t)F(i, j);
    }
}
} // namespace wmtk