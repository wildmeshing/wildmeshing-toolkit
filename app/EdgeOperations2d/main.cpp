#include <EdgeOperations2d.h>
#include <igl/is_edge_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/writeDMAT.h>
#include <stdlib.h>
#include <wmtk/TriMesh.h>
#include <cstdlib>
#include <iostream>
#include <wmtk/utils/ManifoldUtils.hpp>
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
    m.adaptive_remeshing(len, 2, 1);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    wmtk::logger().info("runtime {}", duration.count());
    m.write_triangle_mesh(fmt::format("{}_{}.obj", output, len));
    wmtk::logger().info("current_memory {}", getCurrentRSS() / (1024. * 1024));
    wmtk::logger().info("peak_memory {}", getPeakRSS() / (1024. * 1024));
    wmtk::logger().info(
        "After_vertices#: {} \n\t After_tris#: {}",
        m.vert_capacity(),
        m.tri_capacity());
}

void run_shortest_collapse(std::string input, int target, std::string output, EdgeOperations2d& m)
{
    auto start = high_resolution_clock::now();
    wmtk::logger().info("target number of verts: {}", target);
    m.collapse_shortest(target);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    wmtk::logger().info("runtime {}", duration.count());
    m.consolidate_mesh();
    m.write_triangle_mesh(fmt::format("{}_{}.obj", output, target));
    wmtk::logger().info(
        "After_vertices#: {} \n After_tris#: {}",
        m.vert_capacity(),
        m.tri_capacity());
}

int main(int argc, char** argv)
{
    // 1 path
    // 2 output
    // 3 n_edges
    // 4 eps
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + argv[1];
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    const Eigen::MatrixXd box_min = V.colwise().minCoeff();
    const Eigen::MatrixXd box_max = V.colwise().maxCoeff();
    const double diag = (box_max - box_min).norm();


    const double envelope_size = atof(argv[4]) * diag;
    wmtk::logger().info("readin mesh is {}", ok);
    wmtk::logger().info("envelope_size {}", envelope_size);
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
    m.create_mesh(V.rows(), tri, envelope_size);
    assert(m.check_mesh_connectivity_validity());
    std::vector<double> properties = m.average_len_valen();
    wmtk::logger().info(
        "edgelen: avg max min valence:avg max min before remesh is: {}",
        properties);
    // double small = properties[0] * 0.1;

    // run(path, properties[0] * 0.5, argv[2], m);
    run_shortest_collapse(path, std::stoi(argv[2]), argv[3], m);
    // m.collapse_shortest(atoi(argv[3]));
    // m.write_triangle_mesh(std::string(argv[2]));

    return 0;
}
