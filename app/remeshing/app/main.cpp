#include <remeshing/UniformRemeshing.h>

#include <CLI/CLI.hpp>

#include <wmtk/utils/Reader.hpp>

#include <igl/Timer.h>
#include <igl/read_triangle_mesh.h>
#include <igl/writeDMAT.h>


#include <stdlib.h>
#include <chrono>
#include <cstdlib>
#include <iostream>

using namespace wmtk;
using namespace app::remeshing;
using namespace std::chrono;

#include <wmtk/utils/getRSS.h>

void run_remeshing(std::string input, double len, std::string output, UniformRemeshing& m, int itrs)
{
    auto start = high_resolution_clock::now();
    wmtk::logger().info("target len: {}", len);
    m.uniform_remeshing(len, itrs);
    // m.consolidate_mesh();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    m.consolidate_mesh();
    m.write_triangle_mesh(output);
    auto properties = m.average_len_valen();
    wmtk::logger().info("runtime in ms {}", duration.count());
    wmtk::logger().info("current_memory {}", getCurrentRSS() / (1024. * 1024));
    wmtk::logger().info("peak_memory {}", getPeakRSS() / (1024. * 1024));
    wmtk::logger().info("after remesh properties: {}", properties);
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

    CLI11_PARSE(app, argc, argv);

    wmtk::logger().info("remeshing on {}", input_path);
    wmtk::logger().info("freeze bnd {}", freeze);
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

    UniformRemeshing m(verts, thread, !sample_envelope);
    m.create_mesh(verts.size(), tris, modified_nonmanifold_v, freeze, envelope_size);

    std::vector<double> properties = m.average_len_valen();
    wmtk::logger().info("before remesh properties: {}", properties);
    if (target_len > 0)
        run_remeshing(input_path, target_len, output, m, itrs);

    else {
        double avg_len = m.average_len_valen()[0];
        double len = diag * len_rel;
        len = (len < avg_len * 5) ? len : avg_len * 5;
        run_remeshing(input_path, len, output, m, itrs);
    }

    return 0;
}
