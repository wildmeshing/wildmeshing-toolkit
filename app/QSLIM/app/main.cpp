#include <qslim/QSLIM.h>

#include <wmtk/utils/ManifoldUtils.hpp>

#include <CLI/CLI.hpp>

#include <igl/Timer.h>
#include <igl/readOFF.h>
#include <igl/read_triangle_mesh.h>
#include <igl/writeDMAT.h>
#include <wmtk/utils/Reader.hpp>

#include <stdlib.h>
#include <chrono>
#include <cstdlib>
#include <iostream>

using namespace wmtk;
using namespace qslim;
using namespace std::chrono;

void run_qslim_collapse(std::string input, int target, std::string output, QSLIM& m)
{
    auto start = high_resolution_clock::now();
    wmtk::logger().info("target number of verts: {}", target);
    assert(m.check_mesh_connectivity_validity());
    wmtk::logger().info("mesh is valid");
    m.collapse_qslim(target);
    wmtk::logger().info("collapsed");
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    wmtk::logger().info("runtime in ms{}", duration.count());
}
int main(int argc, char** argv)
{
    std::string input_path = "";
    std::string output = "out.obj";
    double env_rel = -1;
    double target_pec = 0.1;
    int thread = 1;
    double target_verts_percent = 0.1;

    CLI::App app{argv[0]};
    app.add_option("input", input_path, "Input mesh.")->check(CLI::ExistingFile);
    app.add_option("output", output, "output mesh.");

    app.add_option("-t, --target", target_pec, "Percentage of input vertices in output.");
    app.add_option("-e,--envelope", env_rel, "Relative envelope size, negative to disable");
    app.add_option("-j, --thread", thread, "thread.");
    CLI11_PARSE(app, argc, argv);

    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax;
    double remove_duplicate_esp = 1e-5;
    std::vector<size_t> modified_nonmanifold_v;
    wmtk::stl_to_manifold_wmtk_input(
        input_path,
        remove_duplicate_esp,
        box_minmax,
        verts,
        tris,
        modified_nonmanifold_v);

    double diag = (box_minmax.first - box_minmax.second).norm();
    const double envelope_size = env_rel * diag;

    igl::Timer timer;
    timer.start();
    QSLIM m(verts, thread);
    m.create_mesh(verts.size(), tris, modified_nonmanifold_v, envelope_size);
    assert(m.check_mesh_connectivity_validity());
    wmtk::logger().info("collapsing mesh {}", input_path);
    int target_verts = verts.size() * target_pec;

    run_qslim_collapse(input_path, target_verts, output, m);
    timer.stop();
    logger().info("Took {}", timer.getElapsedTimeInSec());
    m.consolidate_mesh();
    m.write_triangle_mesh(output);
    wmtk::logger().info(
        "After_vertices#: {} \n After_tris#: {}",
        m.vert_capacity(),
        m.tri_capacity());
    return 0;
}
