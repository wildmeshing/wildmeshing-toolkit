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
    params.lr = json_params["length_rel"];
    params.stop_energy = json_params["stop_energy"];

    // convert image into tet mesh
    EmbedSurface image_mesh(input_paths[0]);
    {
        if (!skip_simplify) {
            image_mesh.simplify_surface();
        }
        image_mesh.remove_duplicates();
        image_mesh.embed_surface();

        image_mesh.write_emb_msh("debug_input_embedding.msh");
        image_mesh.write_emb_vtu("debug_input_embedding.vtu");
        image_mesh.write_surf_off("debug_surf.off");
        image_mesh.write_emb_surf_off("debug_emb_surf.off");
    }

    // /////////
    // // Prepare Envelope and parameter for TetWild
    // /////////

    const auto box_minmax = image_mesh.bbox_minmax();
    params.init(box_minmax.first, box_minmax.second);

    std::shared_ptr<Envelope> ptr_env;
    std::shared_ptr<SampleEnvelope> ptr_sample_env;
    {
        const auto v_simplified = image_mesh.V_surf_to_vector();
        const auto f_simplified = image_mesh.F_surf_to_vector();

        std::vector<Eigen::Vector3i> tempF(f_simplified.size());
        for (size_t i = 0; i < tempF.size(); ++i) {
            tempF[i] << f_simplified[i][0], f_simplified[i][1], f_simplified[i][2];
        }

        ptr_sample_env = std::make_shared<SampleEnvelope>();
        ptr_sample_env->init(v_simplified, tempF, 0.5); // TODO eps as input parameter

        if (use_sample_envelope) {
            ptr_env = ptr_sample_env;
        } else {
            ptr_env = std::make_shared<ExactEnvelope>();
            ptr_env->init(v_simplified, tempF, 0.5); // TODO eps as input parameter
        }
    }

    /////////////////////////////////////////////////////

    igl::Timer timer;
    timer.start();

    image_simulation::ImageSimulationMesh mesh(params, *ptr_env, *ptr_sample_env, NUM_THREADS);
    mesh.init_from_image(
        image_mesh.V_emb(),
        image_mesh.T_emb(),
        image_mesh.F_on_surface(),
        image_mesh.T_tags());

    // const double insertion_time = insertion_timer.getElapsedTime();
    // wmtk::logger().info("volume remesher insertion time: {}s", insertion_time);

    mesh.consolidate_mesh();
    {
        int num_parts = mesh.flood_fill();
        logger().info("flood fill parts {}", num_parts);
    }

    mesh.write_vtu(params.output_path + "_0.vtu");

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
    std::ofstream fout(params.output_path + ".log");
    fout << "#t: " << mesh.tet_size() << std::endl;
    fout << "#v: " << mesh.vertex_size() << std::endl;
    fout << "max_energy: " << max_energy << std::endl;
    fout << "avg_energy: " << avg_energy << std::endl;
    fout << "eps: " << params.eps << std::endl;
    fout << "threads: " << NUM_THREADS << std::endl;
    fout << "time: " << time << std::endl;
    // fout << "insertion and preprocessing" << insertion_time << std::endl;
    fout.close();

    wmtk::logger().info("final max energy = {} avg = {}", max_energy, avg_energy);
    mesh.write_msh(params.output_path + "_final.msh");
    mesh.write_vtu(params.output_path + "_1.vtu");
    mesh.write_surface(params.output_path + "_surface.obj");

    wmtk::logger().info("======= finish =========");
}

} // namespace wmtk::components::image_simulation