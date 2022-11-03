#include <igl/Timer.h>
#include <regex>
#include "Parameters.h"
#include "TriWild.h"

#include <spdlog/common.h>
#include <CLI/CLI.hpp>

#include <igl/readMSH.h>
#include <igl/read_triangle_mesh.h>

#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

int main(int argc, char** argv)
{
    ZoneScopedN("triwildmain");

    CLI::App app{argv[0]};
    std::string input_file = "./";
    std::string output_file = "./";
    double target_l = -1;

    double target_lr = 0.01;

    double epsr = -1.;
    bool bnd_freeze = false;
    int max_itr = 2;
    double target_e = 4.;
    app.add_option("-i,--input", input_file, "Input mesh.");
    app.add_option("-o,--output", output_file, "Output mesh.");
    app.add_option("--target_l", target_l, "target edge length");
    app.add_option("--target_lr", target_lr, "target edge length");
    app.add_option("--target_e", target_e, "target avg energy");
    app.add_option("--epsr", epsr, "relative envelop size wrt bbox diag");
    app.add_option("--max_itr", max_itr, "number of iterations for improvement");
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
    std::string output_file1 = output_file + output + ".obj";

    // create the json file to record logs
    std::ofstream js_o(output + ".json");

    std::pair<Eigen::VectorXd, Eigen::VectorXd> box_minmax;
    box_minmax = std::pair(V.colwise().minCoeff(), V.colwise().maxCoeff());
    double diag = (box_minmax.first - box_minmax.second).norm();

    // if (target_l < 0) target_l = target_lr * diag;
    igl::Timer timer;
    double time = 0.;
    triwild::TriWild triwild;

    triwild.js_log["input"] = input_file;
    triwild.js_log["output"] = output_file1;

    triwild.m_target_l = target_lr * diag;
    triwild.m_bnd_freeze = bnd_freeze;
    triwild.m_eps = epsr * diag;
    triwild.m_stop_energy = target_e;
    triwild.create_mesh(V, F, epsr * diag, bnd_freeze);
    assert(triwild.check_mesh_connectivity_validity());
    triwild.js_log["num_vert"] = V.rows();
    triwild.js_log["num_faces"] = F.rows();

    triwild.js_log["bbox_diag"] = diag;
    triwild.js_log["edge_length_target"] = triwild.m_target_l;
    triwild.js_log["energy_stop_criteria"] = triwild.m_stop_energy;
    triwild.js_log["freeze_boundary"] = triwild.m_bnd_freeze;
    triwild.js_log["envelop_enabled"] = triwild.m_has_envelope;
    triwild.js_log["improvement_itrs"] = max_itr;

    // get the aabb tree for closest point detect in smooth projection
    // !!!! notice!!!!!
    // arguments and parameters relating to the ABBB has to match the type exactly
    // !!!!!!!!!!!!!!!!
    Eigen::Matrix<uint64_t, Eigen::Dynamic, 2, Eigen::RowMajor> E = triwild.get_bnd_edge_matrix();
    Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor> V_aabb = V.block(0, 0, V.rows(), 2);
    lagrange::bvh::EdgeAABBTree<
        Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>,
        Eigen::Matrix<uint64_t, Eigen::Dynamic, 2, Eigen::RowMajor>,
        2>
        aabb(V_aabb, E);
    triwild.m_get_closest_point = [&aabb](const Eigen::RowVector2d& p) -> Eigen::RowVector2d {
        uint64_t ind = 0;
        double distance = 0.0;
        static Eigen::RowVector2d p_ret;
        aabb.get_closest_point(p, ind, p_ret, distance);
        return p_ret;
    };
    auto energies = triwild.get_quality_all_triangles();
    double start_energy = energies.mean();
    triwild.js_log["energy_start_avg"] = start_energy;
    assert(start_energy > 0);
    int max_idx = energies.maxCoeff();
    triwild.m_max_energy = energies(max_idx);

    wmtk::logger().info("/////starting avg enegry: {}", start_energy);
    // Do the mesh optimization

    triwild.mesh_improvement(max_itr);
    triwild.consolidate_mesh();

    time = timer.getElapsedTime();
    wmtk::logger().info("!!!!finished {}!!!!", time);
    triwild.js_log["total_time"] = time;
    triwild.js_log["energy_final_max"] = triwild.m_max_energy;
    energies = triwild.get_quality_all_triangles();
    triwild.js_log["energy_final_avg"] = energies.mean();

    // Save the optimized mesh
    wmtk::logger().info("/////output : {}", output_file1);
    triwild.write_obj(output_file1);
    js_o << std::setw(4) << triwild.js_log << std::endl;
    js_o.close();
    return 0;
}
