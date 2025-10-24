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

    // if input is not MSH perform conversion to MSH and ignore everything else
    {
        const std::filesystem::path input_filename = input_paths[0];
        const std::vector<double> image_dimensions = json_params["image_dimensions"];

        if (input_filename.extension() != ".msh") {
            // convert image to MSH
            if (output_filename.has_extension() && output_filename.extension() != ".msh") {
                logger().warn(
                    "Changing output extension from {} to .msh",
                    output_filename.extension().string());
            }

            output_filename.replace_extension(".msh");
            std::filesystem::path vtu_filename = output_filename;
            vtu_filename.replace_extension(".vtu");

            logger().info(
                "Converting image {} into mesh {}",
                input_paths[0],
                output_filename.string());

            // convert image into tet mesh
            EmbedSurface image_mesh(input_paths[0], image_dimensions);

            // init params
            {
                const auto& V_input = image_mesh.V_surface();
                params.init(V_input.colwise().minCoeff(), V_input.colwise().maxCoeff());
            }

            if (!skip_simplify) {
                image_mesh.simplify_surface(params.eps);
            }
            image_mesh.remove_duplicates(params.eps);
            image_mesh.embed_surface();
            image_mesh.consolidate();

            image_mesh.write_emb_msh(output_filename.string());
            if (write_vtu) {
                image_mesh.write_emb_vtu(vtu_filename.string());
                image_mesh.write_surf_off("debug_surf.off");
                image_mesh.write_emb_surf_off("debug_emb_surf.off");
            }

            wmtk::logger().info("======= finish conversion =========");
            return;
        }
    }

    output_filename.replace_extension("");


    // read MSH
    MatrixXd V_input;
    MatrixXi T_input;
    VectorXi T_input_tag;
    {
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
    mesh.init_from_image(V_input, T_input, T_input_tag);

    mesh.consolidate_mesh();
    {
        int num_parts = mesh.flood_fill();
        logger().info("flood fill parts {}", num_parts);
    }

    mesh.write_vtu(output_filename.string() + "_0.vtu");

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
    mesh.write_vtu(output_filename.string() + "_1.vtu");
    mesh.write_surface(output_filename.string() + "_surface.obj");

    wmtk::logger().info("======= finish =========");
}

} // namespace wmtk::components::image_simulation