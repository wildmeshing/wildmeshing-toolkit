#include <igl/Timer.h>
#include <spdlog/common.h>
#include <wmtk/utils/Energy2d.h>
#include <CLI/CLI.hpp>
#include <regex>
#include "Parameters.h"
#include "TriWild.h"

#include <igl/readMSH.h>
#include <igl/read_triangle_mesh.h>

#include <lagrange/utils/fpe.h>
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

template <class T>
using RowMatrix2 = Eigen::Matrix<T, Eigen::Dynamic, 2, Eigen::RowMajor>;
using Index = uint64_t;
using Scalar = double;
using namespace wmtk;

int main(int argc, char** argv)
{
    ZoneScopedN("triwildmain");

    lagrange::enable_fpe();
    CLI::App app{argv[0]};
    std::string input_file = "./";
    std::string output_file = "./";
    double target_l = -1;

    double target_lr = 0.01;

    double epsr = -1.;
    bool bnd_freeze = false;
    int max_itr = 2;
    double target_e = 4.;
    triwild::ENERGY_TYPE energy_type = triwild::AMIPS; // "AMIPS" or "SymDi"
    app.add_option("-i,--input", input_file, "Input mesh.");
    app.add_option("-o,--output", output_file, "Output mesh.");
    app.add_option("--target_l", target_l, "target edge length");
    app.add_option("--target_lr", target_lr, "target edge length");
    app.add_option("--target_e", target_e, "target avg energy");
    app.add_option("--epsr", epsr, "relative envelop size wrt bbox diag");
    app.add_option("--max_itr", max_itr, "number of iterations for improvement");
    app.add_option("--bnd_freeze", bnd_freeze, "freeze boundary");
    // app.add_option("--energy_type", energy_type, "enery type");
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

    triwild.mesh_parameters.js_log["input"] = input_file;
    triwild.mesh_parameters.js_log["output"] = output_file1;

    triwild.mesh_parameters.m_target_l = target_lr * diag;
    triwild.mesh_parameters.m_bnd_freeze = bnd_freeze;
    triwild.mesh_parameters.m_eps = epsr * diag;
    triwild.mesh_parameters.m_stop_energy = target_e;
    triwild.mesh_parameters.m_eps = epsr * diag;
    triwild.mesh_parameters.m_bnd_freeze = bnd_freeze;
    triwild.create_mesh(V, F);

    triwild.set_projection();
    triwild.set_energy(std::make_unique<wmtk::AMIPS>());

    if (energy_type == triwild::SYMDI) {
        triwild.set_energy(std::make_unique<wmtk::SymDi>());
    }

    assert(triwild.check_mesh_connectivity_validity());
    triwild.mesh_parameters.js_log["num_vert"] = V.rows();
    triwild.mesh_parameters.js_log["num_faces"] = F.rows();

    triwild.mesh_parameters.js_log["bbox_diag"] = diag;
    triwild.mesh_parameters.js_log["edge_length_target"] = triwild.mesh_parameters.m_target_l;
    triwild.mesh_parameters.js_log["energy_stop_criteria"] = triwild.mesh_parameters.m_stop_energy;
    triwild.mesh_parameters.js_log["freeze_boundary"] = triwild.mesh_parameters.m_bnd_freeze;
    triwild.mesh_parameters.js_log["envelop_enabled"] = triwild.mesh_parameters.m_has_envelope;
    triwild.mesh_parameters.js_log["improvement_itrs"] = max_itr;

    // get the aabb tree for closest point detect in smooth projection
    // !!!! notice!!!!!
    // arguments and parameters relating to the ABBB has to match the type exactly
    // !!!!!!!!!!!!!!!!

    auto energies = triwild.get_quality_all_triangles();
    double start_energy = energies.mean();
    triwild.mesh_parameters.js_log["energy_start_avg"] = start_energy;
    assert(start_energy > 0);
    int max_idx = energies.maxCoeff();
    triwild.mesh_parameters.m_max_energy = energies(max_idx);

    wmtk::logger().info("/////starting avg enegry: {}", start_energy);
    // Do the mesh optimization

    triwild.mesh_improvement(max_itr);
    triwild.consolidate_mesh();

    time = timer.getElapsedTime();
    wmtk::logger().info("!!!!finished {}!!!!", time);
    triwild.mesh_parameters.js_log["total_time"] = time;
    triwild.mesh_parameters.js_log["energy_final_max"] = triwild.mesh_parameters.m_max_energy;
    energies = triwild.get_quality_all_triangles();
    triwild.mesh_parameters.js_log["energy_final_avg"] = energies.mean();

    // Save the optimized mesh
    wmtk::logger().info("/////output : {}", output_file1);
    triwild.write_obj(output_file1);
    js_o << std::setw(4) << triwild.mesh_parameters.js_log << std::endl;
    js_o.close();
    return 0;
}
