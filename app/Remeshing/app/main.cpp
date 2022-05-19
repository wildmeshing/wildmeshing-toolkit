#include <remeshing/UniformRemeshing.h>

#include <CLI/CLI.hpp>

#include <wmtk/utils/ManifoldUtils.hpp>
#include <wmtk/utils/Reader.hpp>

#include <igl/Timer.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/remove_unreferenced.h>
#include <igl/resolve_duplicated_faces.h>
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
    std::string input_path = "";
    std::string output = "out.obj";
    double env_rel = -1;
    double len_rel = 5;
    int thread = 1;
    double target_len = -1;
    int itrs = 2;
    bool freeze = true;
    bool bnd_output = false;
    bool sample_envelope = false;

    CLI::App app{argv[0]};
    app.add_option("input", input_path, "Input mesh.")->check(CLI::ExistingFile);
    app.add_option("output", output, "output mesh.");

    app.add_option("-e,--envelope", env_rel, "Relative envelope size, negative to disable");
    app.add_option("-j, --thread", thread, "thread.");
    app.add_option("-r, --relativelength", len_rel, "Relative edge length.");
    app.add_option("-a, --absolutelength", target_len, "absolute edge length.");
    app.add_option("-i, --iterations", itrs, "number of remeshing itrs.");
    app.add_option("-f, --freeze", freeze, "to freeze the boundary, default to true");
    app.add_flag("--sample-envelope", sample_envelope, "use sample envelope, default to false.");

    app.add_option(
        "--bnd_output",
        bnd_output,
        "(Debug use) write out a table tha maps bnd vertices between original input and output");
    CLI11_PARSE(app, argc, argv);

    wmtk::logger().info("remeshing on {}", input_path);
    wmtk::logger().info("freeze bnd {}", freeze);

    Eigen::MatrixXd inV, V;
    Eigen::MatrixXi inF, F;
    wmtk::reader(input_path, inV, inF);
    Eigen::VectorXi _I;

    igl::remove_unreferenced(inV, inF, V, F, _I);

    if (V.rows() == 0 || F.rows() == 0) {
        wmtk::logger().info("== finish with Empty Input, stop.");
        return 1;
    }

    const Eigen::MatrixXd box_min = V.colwise().minCoeff();
    const Eigen::MatrixXd box_max = V.colwise().maxCoeff();
    const double diag = (box_max - box_min).norm();


    // using the same error tolerance as in tetwild
    Eigen::VectorXi SVI, SVJ, SVK;
    Eigen::MatrixXd temp_V = V; // for STL file
    igl::remove_duplicate_vertices(temp_V, 1e-5 * diag, V, SVI, SVJ);
    for (int i = 0; i < F.rows(); i++)
        for (int j : {0, 1, 2}) F(i, j) = SVJ[F(i, j)];
    auto F1 = F;

    igl::resolve_duplicated_faces(F1, F, SVK);


    std::vector<Eigen::Vector3d> verts(V.rows());
    std::vector<std::array<size_t, 3>> tris(F.rows());
    wmtk::input_formatter(verts, tris, V, F);

    wmtk::logger().info("Before_vertices#: {} \n Before_tris#: {}", V.rows(), F.rows());

    const double envelope_size = env_rel * diag;
    Eigen::VectorXi dummy;
    std::vector<size_t> modified_v;
    if (!igl::is_edge_manifold(F) || !igl::is_vertex_manifold(F, dummy)) {
        auto v1 = verts;
        auto tri1 = tris;
        wmtk::separate_to_manifold(v1, tri1, verts, tris, modified_v);
    }

    igl::Timer timer;
    timer.start();
    UniformRemeshing m(verts, thread, !sample_envelope);
    m.create_mesh(verts.size(), tris, modified_v, freeze, envelope_size);

    if (bnd_output) m.get_boundary_map(SVI);

    // std::vector<double> properties = m.average_len_valen();
    // wmtk::logger().info(
    //     "edgelen: avg max min valence:avg max min before remesh is: {}",
    //     properties);
    if (target_len > 0)
        run_remeshing(input_path, target_len, output, m, itrs, bnd_output);

    else {
        double avg_len = m.average_len_valen()[0];
        double len = diag * len_rel;
        len = (len < avg_len * 5) ? len : avg_len * 5;
        run_remeshing(input_path, len, output, m, itrs, bnd_output);
    }
    timer.stop();
    logger().info("Took {}", timer.getElapsedTimeInSec());

    return 0;
}
