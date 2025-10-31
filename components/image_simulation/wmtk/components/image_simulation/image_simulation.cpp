#include "image_simulation.hpp"


#include <memory>
#include <vector>

#include <geogram/mesh/mesh_io.h>
#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Partitioning.h>
#include <paraviewo/VTUWriter.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/utils/InsertTriangleUtils.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>
#include <wmtk/utils/Reader.hpp>
#include <wmtk/utils/io.hpp>
#include <wmtk/utils/partition_utils.hpp>

#include <sec/ShortestEdgeCollapse.h>

#include "EmbedSurface.hpp"
#include "ImageSimulationMesh.h"
#include "Parameters.h"
#include "extract_soup.hpp"

#include "image_simulation_spec.hpp"

namespace wmtk::components::image_simulation {

void image_simulation(nlohmann::json json_params)
{
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

    // logger settings
    {
        std::string log_file_name = json_params["log_file"];
        if (!log_file_name.empty()) {
            wmtk::set_file_logger(log_file_name);
            logger().flush_on(spdlog::level::info);
        }
    }

    GEO::Process::enable_multithreading(false);

    std::vector<std::string> input_paths = json_params["input"];

    Parameters params;
    params.output_path = json_params["output"];
    bool skip_simplify = json_params["skip_simplify"];
    bool use_sample_envelope = json_params["use_sample_envelope"];
    int NUM_THREADS = json_params["num_threads"];
    int max_its = json_params["max_iterations"];

    params.epsr = json_params["eps_rel"];
    params.eps = json_params["eps"];
    params.lr = json_params["length_rel"];
    params.stop_energy = json_params["stop_energy"];

    const bool write_vtu = json_params["write_vtu"];

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
        return fmt::format("{}_{}.vtu", output_filename.string(), vtu_counter++);
    };

    // read image / MSH
    MatrixXd V_input;
    MatrixXr V_input_r;
    MatrixXi T_input;
    VectorXi T_input_tag;

    if (std::filesystem::path(input_paths[0]).extension() != ".msh") {
        // convert image to tet mesh

        const std::vector<double> image_spacing = json_params["image_spacing"];

        logger().info(
            "Converting image {} into mesh {}",
            input_paths[0],
            output_filename.string() + ".msh");

        // convert image into tet mesh
        EmbedSurface image_mesh(input_paths[0], image_spacing);

        const double eps = *(std::min_element(image_spacing.begin(), image_spacing.end())) * 0.1;
        if (!skip_simplify) {
            logger().info("Simplify...");
            image_mesh.simplify_surface(eps);
            logger().info("done");
        }
        image_mesh.remove_duplicates(eps);
        const bool all_rounded = image_mesh.embed_surface();
        image_mesh.consolidate();

        if (write_vtu) {
            image_mesh.write_emb_vtu(get_unique_vtu_name());
            image_mesh.write_surf_off(output_filename.string() + "_input.off");
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
        MshData msh;
        msh.load(input_paths[0]);

        std::vector<Eigen::Vector3d> verts;
        std::vector<std::array<size_t, 4>> tets;
        std::vector<int> tets_tag;

        verts.resize(msh.get_num_tet_vertices());
        tets.resize(msh.get_num_tets());
        msh.extract_tet_vertices(
            [&verts](size_t i, double x, double y, double z) { verts[i] << x, y, z; });
        msh.extract_tets([&tets](size_t i, size_t v0, size_t v1, size_t v2, size_t v3) {
            tets[i] = {{v0, v1, v2, v3}};
        });

        tets_tag.resize(msh.get_num_tets());
        msh.extract_tet_attribute("tag", [&tets_tag](size_t i, std::vector<double> val) {
            assert(val.size() == 1);
            tets_tag[i] = val[0];
        });

        assert(tets.size() == tets_tag.size());

        V_input = V_MAP(verts[0].data(), verts.size(), 3);
        T_input.resize(tets.size(), 4);
        T_input_tag.resize(tets_tag.size(), 1);
        for (size_t i = 0; i < tets.size(); ++i) {
            T_input(i, 0) = tets[i][0];
            T_input(i, 1) = tets[i][1];
            T_input(i, 2) = tets[i][2];
            T_input(i, 3) = tets[i][3];
            T_input_tag[i] = tets_tag[i];
        }
    }

    params.init(V_input.colwise().minCoeff(), V_input.colwise().maxCoeff());

    igl::Timer timer;
    timer.start();

    image_simulation::ImageSimulationMesh mesh(params, params.eps, NUM_THREADS);
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