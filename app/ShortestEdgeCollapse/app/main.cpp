#include <igl/is_edge_manifold.h>
#include <igl/readOFF.h>
#include <igl/read_triangle_mesh.h>
#include <igl/writeDMAT.h>
#include <sec/ShortestEdgeCollapse.h>
#include <stdlib.h>
#include <wmtk/TriMesh.h>
#include <cstdlib>
#include <iostream>
#include <wmtk/utils/ManifoldUtils.hpp>
using namespace wmtk;

using namespace sec;
#include <chrono>
using namespace std::chrono;

extern "C" {
#include <wmtk/utils/getRSS.c>
};
void run_shortest_collapse(
    std::string input,
    int target,
    std::string output,
    ShortestEdgeCollapse& m)
{
    auto start = high_resolution_clock::now();
    wmtk::logger().info("target number of verts: {}", target);
    assert(m.check_mesh_connectivity_validity());
    wmtk::logger().info("mesh is valid");
    m.collapse_shortest(target);
    wmtk::logger().info("collapsed");
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
    // input
    // target_verts
    // output
    // ep

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

    const Eigen::MatrixXd box_min = V.colwise().minCoeff();
    const Eigen::MatrixXd box_max = V.colwise().maxCoeff();
    const double diag = (box_max - box_min).norm();

    const double envelope_size = atof(argv[4]) * diag;

    if (!igl::is_edge_manifold(F)) {
        return 1;
    }

    else {
        ShortestEdgeCollapse m(v);
        m.create_mesh(v.size(), tri, envelope_size);
        assert(m.check_mesh_connectivity_validity());
        wmtk::logger().info("collapsing mesh {}", argv[1]);
        run_shortest_collapse(path, std::stoi(argv[2]), argv[3], m);
        m.consolidate_mesh();
    }
    return 0;
}
