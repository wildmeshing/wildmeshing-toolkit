#include <EdgeOperations2d.h>
#include <igl/read_triangle_mesh.h>
#include <stdlib.h>
#include <wmtk/TriMesh.h>
#include <cstdlib>
#include <iostream>
#include <wmtk/utils/getRSS.c>
using namespace wmtk;

using namespace Edge2d;
#include <chrono>
using namespace std::chrono;

// extern "C" size_t getPeakRSS();
int main(int argc, char** argv)
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + argv[1];
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);
    wmtk::logger().info("Before_vertices#: {} \n Before_tris#: {}", V.rows(), F.rows());

    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }
    EdgeOperations2d m(v);
    m.create_mesh(V.rows(), tri);
    assert(m.check_mesh_connectivity_validity());

    auto start = high_resolution_clock::now();
    m.adaptive_remeshing(std::stod(argv[2]), std::stod(argv[3]), std::stoi(argv[5]));
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    wmtk::logger().info("runtime {}", duration.count());
    m.write_triangle_mesh(argv[4]);
    wmtk::logger().info("peak_memory {}", getCurrentRSS() / (1024 * 1024));
    wmtk::logger().info(
        "After_vertices#: {} \n After_tris#: {}",
        m.vert_capacity(),
        m.tri_capacity());


    return 0;
}