#include <sec/ShortestEdgeCollapse.h>

#include <wmtk/utils/ManifoldUtils.hpp>

#include <CLI/CLI.hpp>

#include <igl/Timer.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/readOFF.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/writeDMAT.h>

#include <stdlib.h>
#include <chrono>
#include <cstdlib>
#include <iostream>

using namespace wmtk;
using namespace app::sec;
using namespace std::chrono;

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
    // m.consolidate_mesh();
    m.write_triangle_mesh(output);
    wmtk::logger().info(
        "After_vertices#: {} \n After_tris#: {}",
        m.vert_capacity(),
        m.tri_capacity());
}

int main(int argc, char** argv)
{
    std::string path = "";
    std::string output = "out.obj";
    double env_rel = -1;
    double target_pec = 0.1;
    int thread = 1;

    CLI::App app{argv[0]};
    app.add_option("input", path, "Input mesh.")->check(CLI::ExistingFile);
    app.add_option("output", output, "output mesh.");

    app.add_option("-e,--envelope", env_rel, "Relative envelope size, negative to disable");
    app.add_option("-j, --thread", thread, "thread.");
    app.add_option("-t, --target", target_pec, "Percentage of input vertices in output.");
    CLI11_PARSE(app, argc, argv);

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    Eigen::VectorXi SVI, SVJ;
    Eigen::MatrixXd temp_V = V; // for STL file
    igl::remove_duplicate_vertices(temp_V, 0, V, SVI, SVJ);
    for (int i = 0; i < F.rows(); i++)
        for (int j : {0, 1, 2}) F(i, j) = SVJ[F(i, j)];

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

    const double envelope_size = env_rel * diag;
    Eigen::VectorXi dummy;
    std::vector<size_t> modified_v;
    if (!igl::is_edge_manifold(F) || !igl::is_vertex_manifold(F, dummy)) {
        auto v1 = v;
        auto tri1 = tri;
        wmtk::separate_to_manifold(v1, tri1, v, tri, modified_v);
    }

    ShortestEdgeCollapse m(v, thread);
    m.create_mesh(v.size(), tri, modified_v, envelope_size);
    assert(m.check_mesh_connectivity_validity());
    wmtk::logger().info("collapsing mesh {}", path);
    int target_verts = v.size() * target_pec;
    igl::Timer timer;
    timer.start();
    run_shortest_collapse(path, target_verts, output, m);
    timer.stop();
    logger().info("Took {}", timer.getElapsedTimeInSec());
    // m.consolidate_mesh();
    return 0;
}
