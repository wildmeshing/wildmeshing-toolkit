#include "Parameters.h"
#include "TriWild.h"

#include <spdlog/common.h>
#include <CLI/CLI.hpp>

#include <igl/read_triangle_mesh.h>

int main(int argc, char** argv)
{
    ZoneScopedN("triwildmain");

    // Parsing of parameters
    triwild::Parameters params;

    CLI::App app{argv[0]};
    std::string input_file = "./";
    std::string output_file = "./";

    app.add_option("-i,--input", input_file, "Input mesh.");
    app.add_option("-o,--output", output_file, "Output mesh.");
    // app.add_option("-j,--jobs", NUM_THREADS, "thread.");

    CLI11_PARSE(app, argc, argv);

    // Loading the input mesh
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(input_file, V, F);

    assert(ok);

    // Load the mesh in the trimesh class
    triwild::TriWild triwild;
    triwild.create_mesh(V, F);
    assert(triwild.check_mesh_connectivity_validity());

    // Do the mesh optimization
    // triwild.optimize();
    triwild.consolidate_mesh();

    // Save the optimized mesh
    triwild.write_obj(output_file);

    // Output
    // auto [max_energy, avg_energy] = mesh.get_max_avg_energy();
    // std::ofstream fout(output_file + ".log");
    // fout << "#t: " << mesh.tet_size() << std::endl;
    // fout << "#v: " << mesh.vertex_size() << std::endl;
    // fout << "max_energy: " << max_energy << std::endl;
    // fout << "avg_energy: " << avg_energy << std::endl;
    // fout << "eps: " << params.eps << std::endl;
    // fout << "threads: " << NUM_THREADS << std::endl;
    // fout << "time: " << time << std::endl;
    // fout.close();

    // igl::write_triangle_mesh(output_path + "_surface.obj", matV, matF);
    // wmtk::logger().info("Output face size {}", outface.size());
    // wmtk::logger().info("======= finish =========");

    return 0;
}