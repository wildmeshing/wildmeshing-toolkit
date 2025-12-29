#include "image_simulation.hpp"


#include <memory>
#include <vector>

#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include "EmbedSurface.hpp"
#include "ImageSimulationMesh.h"
#include "Parameters.h"
#include "extract_soup.hpp"
#include "read_image_msh.hpp"

#include "image_simulation_spec.hpp"

// Enables passing Eigen matrices to fmt/spdlog.
template <typename T>
struct fmt::formatter<T, std::enable_if_t<std::is_base_of_v<Eigen::DenseBase<T>, T>, char>>
    : ostream_formatter
{
};


namespace wmtk::components::image_simulation {

void image_simulation(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;
    using Tuple = TetMesh::Tuple;

    // verify input and inject defaults
    {
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, image_simulation_spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, image_simulation_spec);
    }

    const std::filesystem::path root = json_params["json_input_file"];

    // logger settings
    {
        std::string log_file_name = json_params["log_file"];
        if (!log_file_name.empty()) {
            log_file_name = resolve_path(root, log_file_name).string();
            wmtk::set_file_logger(log_file_name);
            logger().flush_on(spdlog::level::info);
        }
    }

    std::vector<std::string> input_paths = json_params["input"];
    for (std::string& p : input_paths) {
        p = resolve_path(root, p).string();
    }

    Parameters params;
    params.output_path = resolve_path(root, json_params["output"]).string();
    bool skip_simplify = json_params["skip_simplify"];
    bool use_sample_envelope = json_params["use_sample_envelope"];
    int NUM_THREADS = json_params["num_threads"];
    int max_its = json_params["max_iterations"];

    params.epsr = json_params["eps_rel"];
    params.eps = json_params["eps"];
    params.lr = json_params["length_rel"];
    params.stop_energy = json_params["stop_energy"];
    params.preserve_topology = json_params["preserve_topology"];

    const bool write_vtu = json_params["write_vtu"];

    params.debug_output = json_params["DEBUG_output"];
    params.perform_sanity_checks = json_params["DEBUG_sanity_checks"];

    std::filesystem::path output_filename = params.output_path;

    if (output_filename.has_extension() && output_filename.extension() != ".msh") {
        output_filename.replace_extension(".msh");
        logger().warn(
            "Extension of provided output filename is ignored. Output will be {}",
            output_filename.string());
    }
    output_filename.replace_extension(""); // extension is added back later

    auto get_unique_vtu_name = [&output_filename]() -> std::string {
        static size_t vtu_counter = 0;
        return fmt::format("{}_{}", output_filename.string(), vtu_counter++);
    };

    // read image / MSH
    MatrixXd V_input;
    MatrixXr V_input_r;
    MatrixXi T_input;
    MatrixXi T_input_tag;

    MatrixXd V_envelope;
    MatrixXi F_envelope;

    if (std::filesystem::path(input_paths[0]).extension() != ".msh") {
        // convert images to tet mesh

        const std::array<std::array<double, 4>, 4> ijk_to_ras = json_params["ijk_to_ras"];
        Matrix4d ijk2ras;
        for (size_t i = 0; i < 4; ++i) {
            for (size_t j = 0; j < 4; ++j) {
                ijk2ras(i, j) = ijk_to_ras[i][j];
            }
        }
        logger().info("IJK to RAS:\n{}", ijk2ras);
        /**
         * R = -x
         * A = z
         * S = y
         */
        Matrix4d ras2xyz;
        ras2xyz << -1, 0, 0, 0, //
            0, 0, 1, 0, //
            0, 1, 0, 0, //
            0, 0, 0, 1;
        Matrix4d ijk2xyz = ras2xyz * ijk2ras;

        logger().info(
            "Converting images {} into mesh {}",
            input_paths,
            output_filename.string() + ".msh");

        const Vector4d single_voxel_max = ijk2xyz * Vector4d::Ones();
        const Vector4d single_voxel_min = ijk2xyz * Vector4d(0, 0, 0, 1);
        double eps = (from_homogenuous(single_voxel_max) - from_homogenuous(single_voxel_min))
                         .cwiseAbs()
                         .minCoeff() *
                     0.1;
        if (eps <= 0) {
            logger().warn("EPS = {}, ijk_to_ras matix might be broken! Changing eps to 1e-4", eps);
            eps = 1e-4;
        }

        // convert image into tet mesh
        EmbedSurface image_mesh(input_paths, ijk2xyz);

        if (!skip_simplify) {
            logger().info("Simplify...");
            image_mesh.simplify_surface(eps);
            logger().info("done");
        }
        image_mesh.remove_duplicates(eps);

        if (write_vtu) {
            image_mesh.write_surf_off(output_filename.string() + "_input.off");
        }

        V_envelope = image_mesh.V_surface();
        F_envelope = image_mesh.F_surface();

        // const bool all_rounded = image_mesh.embed_surface();
        const bool all_rounded = json_params["use_tetgen"] ? image_mesh.embed_surface_tetgen()
                                                           : image_mesh.embed_surface();
        image_mesh.consolidate();

        if (write_vtu) {
            image_mesh.write_emb_vtu(get_unique_vtu_name());
            image_mesh.write_emb_surf_off(output_filename.string() + "_input_emb.off");
        }

        V_input = image_mesh.V_emb();
        if (!all_rounded) {
            V_input_r = image_mesh.V_emb_r();
        }
        T_input = image_mesh.T_emb();
        T_input_tag = image_mesh.T_tags();

        wmtk::logger().info("======= finish image-tet conversion =========");
        // return;
    } else {
        // input is a tet mesh
        read_image_msh(input_paths[0], V_input, T_input, T_input_tag, V_envelope, F_envelope);
    }

    params.init(V_input.colwise().minCoeff(), V_input.colwise().maxCoeff());

    igl::Timer timer;
    timer.start();

    image_simulation::ImageSimulationMesh mesh(params, params.eps, NUM_THREADS);
    // first init envelope
    if (V_envelope.size() != 0) {
        mesh.init_envelope(V_envelope, F_envelope);
    }
    if (V_input_r.size() == 0) {
        logger().info("Use float input for TetWild");
        mesh.init_from_image(V_input, T_input, T_input_tag);
    } else {
        logger().warn("Use RATIONAL input for TetWild");
        mesh.init_from_image(V_input_r, T_input, T_input_tag);
    }

    mesh.consolidate_mesh();
    {
        int num_parts = mesh.flood_fill();
        logger().info("flood fill parts {}", num_parts);
    }

    if (write_vtu) mesh.write_vtu(get_unique_vtu_name());

    //// smooth
    //{
    //    mesh.smooth_input(10);
    //    mesh.write_vtu(params.output_path + "_1.vtu");
    //    return;
    //}


    {
        size_t nonmani_ver_cnt = 0;
        size_t surface_v_cnt = 0;
        for (const Tuple& v : mesh.get_vertices()) {
            if (mesh.m_vertex_attribute[v.vid(mesh)].m_is_on_surface) {
                surface_v_cnt++;
                if (mesh.count_vertex_links(v) > 1) {
                    nonmani_ver_cnt++;
                }
            }
        }

        wmtk::logger().info("MESH NONMANIFOLD VERTEX COUNT BEFORE OPTIMIZE: {}", nonmani_ver_cnt);
        wmtk::logger().info("MESH surface VERTEX COUNT BEFORE OPTIMIZE: {}", surface_v_cnt);
    }

    // /////////mesh improvement
    mesh.mesh_improvement(max_its); // <-- tetwild

    mesh.consolidate_mesh();
    double time = timer.getElapsedTime();
    wmtk::logger().info("total time {}s", time);

    {
        int num_parts = mesh.flood_fill();
        logger().info("flood fill parts {}", num_parts);
    }

    /////////output
    auto [max_energy, avg_energy] = mesh.get_max_avg_energy();
    std::ofstream fout(output_filename.string() + ".log");
    fout << "#t: " << mesh.tet_size() << std::endl;
    fout << "#v: " << mesh.vertex_size() << std::endl;
    fout << "max_energy: " << max_energy << std::endl;
    fout << "avg_energy: " << avg_energy << std::endl;
    fout << "eps: " << params.eps << std::endl;
    fout << "threads: " << NUM_THREADS << std::endl;
    fout << "time: " << time << std::endl;
    fout.close();

    wmtk::logger().info("final max energy = {} avg = {}", max_energy, avg_energy);
    mesh.write_msh(output_filename.string() + ".msh");
    if (write_vtu) mesh.write_vtu(get_unique_vtu_name());
    if (write_vtu) mesh.write_surface(output_filename.string() + "_surface.obj");

    wmtk::logger().info("======= finish =========");
}

} // namespace wmtk::components::image_simulation