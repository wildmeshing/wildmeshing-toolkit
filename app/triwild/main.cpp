#include "Parameters.h"
#include "TriWild.h"
#include <spdlog/common.h>
#include <CLI/CLI.hpp>


int main(int argc, char** argv)
{
    ZoneScopedN("triwildmain");

    // Parsing of parameters
    triwild::Parameters params;

    CLI::App app{argv[0]};
    std::string input_path = WMT_DATA_DIR "/37322.stl";
    std::string output_path = "./";

    app.add_option("-i,--input", input_path, "Input mesh.");
    app.add_option("-o,--output", output_path, "Output mesh.");
    // app.add_option("-j,--jobs", NUM_THREADS, "thread.");

    CLI11_PARSE(app, argc, argv);


    // Output
    // auto [max_energy, avg_energy] = mesh.get_max_avg_energy();
    // std::ofstream fout(output_path + ".log");
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