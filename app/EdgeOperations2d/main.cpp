#include <EdgeOperations2d.h>
#include <igl/read_triangle_mesh.h>
#include <stdlib.h>
#include <wmtk/TriMesh.h>
#include <cstdlib>
#include <iostream>
using namespace wmtk;

using namespace Edge2d;
int main(int argc, char** argv)
{
    // const std::string root(WMT_DATA_DIR);
    const std::string path = argv[1];
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    EdgeOperations2d m(v);
    m.create_mesh(V.rows(), tri, atof(argv[5]));
    assert(m.check_mesh_connectivity_validity());
    m.adaptive_remeshing(std::stod(argv[2]), std::stod(argv[3]));
    m.write_triangle_mesh(argv[4]);
    return 0;
}