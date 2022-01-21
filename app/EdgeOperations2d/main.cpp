#include <EdgeOperations2d.h>
#include <igl/read_triangle_mesh.h>
#include <stdlib.h>
#include <wmtk/TriMesh.h>
#include <cstdlib>
#include <iostream>
using namespace wmtk;

using namespace Edge2d;
#include <chrono>
using namespace std::chrono;

extern "C" {
#include <wmtk/utils/getRSS.c>
};
void run(std::string input, double len, std::string output, EdgeOperations2d& m)
{
    auto start = high_resolution_clock::now();
    wmtk::logger().info("target len: {}", len);
    m.adaptive_remeshing(len, 5, 1);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    wmtk::logger().info("runtime {}", duration.count());
    m.write_triangle_mesh(output + std::string("_") + std::to_string(len) + std::string(".obj"));
    wmtk::logger().info("current_memory {}", getCurrentRSS() / (1024 * 1024));
    wmtk::logger().info("peak_memory {}", getPeakRSS() / (1024 * 1024));
    wmtk::logger().info(
        "After_vertices#: {} \n\t After_tris#: {}",
        m.vert_capacity(),
        m.tri_capacity());
}

int main(int argc, char** argv)
{
    const std::string path = argv[1];
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);
    wmtk::logger().info("readin mesh is {}", ok);
    wmtk::logger().info("Before_vertices#: {} \n\t Before_tris#: {}", V.rows(), F.rows());

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
    std::vector<double> properties = m.average_len_valen();
    wmtk::logger().info(
        "edgelen: avg max min valence:avg max min before remesh is: {}",
        properties);
    double small = properties[0] * 0.1;
    
    run(path, small, std::string(argv[2]), m);

    return 0;
}
