#include <remeshing/UniformRemeshing.h>

#include <wmtk/utils/ManifoldUtils.hpp>

#include <CLI/CLI.hpp>

#include <igl/Timer.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/writeDMAT.h>

#include <stdlib.h>
#include <chrono>
#include <cstdlib>
#include <iostream>

using namespace wmtk;
using namespace remeshing;
using namespace std::chrono;

extern "C" {
#include <wmtk/utils/getRSS.c>
}

void run_remeshing(
    std::string input,
    double len,
    std::string output,
    UniformRemeshing& m,
    int itrs,
    bool bnd_output)
{
    auto start = high_resolution_clock::now();
    wmtk::logger().info("target len: {}", len);
    m.uniform_remeshing(len, itrs);
    m.consolidate_mesh(bnd_output);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    m.consolidate_mesh();
    m.write_triangle_mesh(output);
    wmtk::logger().info("runtime {}", duration.count());
    wmtk::logger().info("current_memory {}", getCurrentRSS() / (1024. * 1024));
    wmtk::logger().info("peak_memory {}", getPeakRSS() / (1024. * 1024));
    wmtk::logger().info(
        "After_vertices#: {} \n\t After_tris#: {}",
        m.vert_capacity(),
        m.tri_capacity());
}

int main(int argc, char** argv)
{
    std::string path = "";
    std::string output = "out.obj";
    double env_rel = -1;
    double len_rel = 5;
    int thread = 1;
    double target_len = -1;
    int itrs = 2;
    bool freeze = true;
    bool bnd_output = false;

    CLI::App app{argv[0]};
    app.add_option("input", path, "Input mesh.")->check(CLI::ExistingFile);
    app.add_option("output", output, "output mesh.");

    app.add_option("-e,--envelope", env_rel, "Relative envelope size, negative to disable");
    app.add_option("-j, --thread", thread, "thread.");
    app.add_option("-r, --relativelength", len_rel, "Relative edge length.");
    app.add_option("-a, --absolutelength", target_len, "absolute edge length.");
    app.add_option("-i, --iterations", itrs, "number of remeshing itrs.");
    app.add_option("-f, --freeze", freeze, "to freeze the boundary, default to true");
    app.add_option(
        "-b, --bnd_output",
        bnd_output,
        "write out a table tha maps bnd vertices between original input and output");
    CLI11_PARSE(app, argc, argv);

    wmtk::logger().info("remeshing on {}", path);
    wmtk::logger().info("freeze bnd {}", freeze);

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);
    Eigen::VectorXi SVI, SVJ;
    Eigen::MatrixXd temp_V = V; // for STL file
    igl::remove_duplicate_vertices(temp_V, 1e-3, V, SVI, SVJ);
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

    UniformRemeshing m(v, thread);
    m.create_mesh(v.size(), tri, modified_v, freeze, envelope_size);

    if (bnd_output) m.get_boundary_map(SVI);

    m.get_vertices();
    std::vector<double> properties = m.average_len_valen();
    wmtk::logger().info(
        "edgelen: avg max min valence:avg max min before remesh is: {}",
        properties);
    igl::Timer timer;
    timer.start();
    if (target_len > 0)
        run_remeshing(path, target_len, output, m, itrs, bnd_output);
    else
        run_remeshing(path, diag * len_rel, output, m, itrs, bnd_output);
    timer.stop();
    logger().info("Took {}", timer.getElapsedTimeInSec());

    return 0;
}
