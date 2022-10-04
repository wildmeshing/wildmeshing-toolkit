#include <regex>

#include "Parameters.h"
#include "TriWild.h"

#include <spdlog/common.h>
#include <CLI/CLI.hpp>

#include <igl/readMSH.h>
#include <igl/read_triangle_mesh.h>

int main(int argc, char** argv)
{
    ZoneScopedN("triwildmain");

    // Parsing of parameters
    triwild::Parameters params;

    CLI::App app{argv[0]};
    std::string input_file = "./";
    std::string output_file = "./";
    double target_l = -1;

    double target_lr1 = 0.1;
    double target_lr2 = 0.001;

    double epsr = -1.;
    bool bnd_freeze = false;
    app.add_option("-i,--input", input_file, "Input mesh.");
    app.add_option("-o,--output", output_file, "Output mesh.");
    app.add_option("--target_l", target_l, "target edge length");
    // app.add_option("--target_lr", target_lr, "target edge length");
    app.add_option("--epsr", epsr, "relative envelop size wrt bbox diag");
    app.add_option("--bnd_freeze", bnd_freeze, "freeze boundary");
    // app.add_option("-j,--jobs", NUM_THREADS, "thread."

    CLI11_PARSE(app, argc, argv);


    // Loading the input mesh
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::readMSH(input_file, V, F);

    assert(ok);
    wmtk::logger().info("/////input: {}", input_file);
    std::string output =
        std::regex_replace(input_file, std::regex("[^0-9]*([0-9]+).*"), std::string("$1"));
    std::string output_file1 = "/home/yunfan/data/triwild_output_lin/output1/" + output + "1.obj";
    std::string output_file2 = "/home/yunfan/data/triwild_output_lin/output2/" + output + "2.obj";

    std::pair<Eigen::VectorXd, Eigen::VectorXd> box_minmax;
    box_minmax = std::pair(V.colwise().minCoeff(), V.colwise().maxCoeff());
    double diag = (box_minmax.first - box_minmax.second).norm();
    // if (target_l < 0) target_l = target_lr * diag;

    triwild::TriWild triwild;
    triwild.m_target_l = target_lr1 * diag;
    triwild.m_bnd_freeze = bnd_freeze;
    triwild.m_eps = epsr * diag;
    triwild.create_mesh(V, F, epsr * diag, bnd_freeze);

    assert(triwild.check_mesh_connectivity_validity());
    double start_energy = triwild.get_quality_all_triangles().mean();
    assert(start_energy > 0);
    wmtk::logger().info("/////starting avg enegry: {}", start_energy);
    // Do the mesh optimization
    // triwild.optimize();
    triwild.mesh_improvement(10);
    triwild.consolidate_mesh();

    // Save the optimized mesh
    wmtk::logger().info("/////output : {}", output_file1);
    triwild.write_obj(output_file1);

    Eigen::MatrixXd V2;
    Eigen::MatrixXi F2;
    ok = igl::read_triangle_mesh(input_file, V2, F2);
    assert(ok);
    wmtk::logger().info("/////input: {}", input_file);

    triwild::TriWild triwild2;
    triwild2.m_target_l = target_lr2 * diag;
    triwild2.m_bnd_freeze = bnd_freeze;
    triwild2.m_eps = epsr * diag;
    triwild2.create_mesh(V2, F2, epsr * diag, bnd_freeze);

    assert(triwild2.check_mesh_connectivity_validity());
    start_energy = triwild2.get_quality_all_triangles().mean();
    wmtk::logger().info("/////starting avg enegry: {}", start_energy);
    // Do the mesh optimization
    // triwild.optimize();
    triwild2.mesh_improvement(10);
    triwild2.consolidate_mesh();

    // Save the optimized mesh
    triwild2.write_obj(output_file2);

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