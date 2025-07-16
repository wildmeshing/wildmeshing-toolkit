#include "image_simulation.hpp"


#include <memory>
#include <vector>

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

    image_simulation::Parameters params;

    std::vector<std::string> input_paths = json_params["input"];
    params.output_path = json_params["output"];
    bool skip_simplify = json_params["skip_simplify"];
    bool use_sample_envelope = json_params["use_sample_envelope"];
    int NUM_THREADS = json_params["num_threads"];
    int max_its = json_params["max_iterations"];
    bool filter_with_input = json_params["filter_with_input"];

    params.epsr = json_params["eps_rel"];
    params.lr = json_params["length_rel"];
    params.stop_energy = json_params["stop_energy"];

    // extract surface from image
    {
        // read raw image data + create triangle soup
        Eigen::MatrixXi F;
        Eigen::MatrixXd V;
        extract_triangle_soup_from_image(input_paths[0], F, V);
        igl::writeOFF("triangle_soup_fine.off", V, F);
    }

    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    std::vector<size_t> modified_nonmanifold_v;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax;

    // read triangle soup
    {
        const double remove_duplicate_eps = 0.01;
        wmtk::stl_to_manifold_wmtk_input(
            "triangle_soup_fine.off",
            remove_duplicate_eps,
            box_minmax,
            verts,
            tris,
            modified_nonmanifold_v);
    }

    std::vector<Eigen::Vector3d> v_simplified;
    std::vector<std::array<size_t, 3>> f_simplified;

    // simplification
    app::sec::ShortestEdgeCollapse surf_mesh(verts, NUM_THREADS, false);
    {
        // must be a small envelope to ensure correct tet tags later on
        surf_mesh.create_mesh(verts.size(), tris, modified_nonmanifold_v, 0.1);
        assert(surf_mesh.check_mesh_connectivity_validity());

        wmtk::logger().info("input {} simplification", input_paths);
        surf_mesh.collapse_shortest(0);
        surf_mesh.consolidate_mesh();
        surf_mesh.write_triangle_mesh("triangle_soup_coarse.off");

        //// get the simplified input
        v_simplified.resize(surf_mesh.vert_capacity());
        f_simplified.resize(surf_mesh.tri_capacity());
        for (const auto& t : surf_mesh.get_vertices()) {
            const size_t i = t.vid(surf_mesh);
            v_simplified[i] = surf_mesh.vertex_attrs[i].pos;
        }

        for (const auto& t : surf_mesh.get_faces()) {
            const auto i = t.fid(surf_mesh);
            const auto vs = surf_mesh.oriented_tri_vids(t);
            for (int j = 0; j < 3; j++) {
                f_simplified[i][j] = vs[j];
            }
        }
    }

    wmtk::remove_duplicates(v_simplified, f_simplified, 0.01);

    // embedding
    {
        const V_MAP V_surface(v_simplified[0].data(), v_simplified.size(), 3);
        MatrixXi F_surface;
        F_surface.resize(f_simplified.size(), 3);
        for (int i = 0; i < f_simplified.size(); ++i) {
            const auto& f = f_simplified[i];
            F_surface.row(i) = Vector3i(f[0], f[1], f[2]);
        }
        std::shared_ptr<Envelope> ptr_env;
        {
            std::vector<Eigen::Vector3i> tempF(f_simplified.size());
            for (size_t i = 0; i < tempF.size(); ++i) {
                tempF[i] << f_simplified[i][0], f_simplified[i][1], f_simplified[i][2];
            }
            ptr_env = std::make_shared<ExactEnvelope>();
            ptr_env->init(v_simplified, tempF, 0.5);
        }

        std::vector<wmtk::delaunay::Point3D> points;
        std::vector<wmtk::delaunay::Tetrahedron> tets;
        Vector3d box_min;
        Vector3d box_max;
        delaunay_box_mesh(*ptr_env, V_surface, points, tets, box_min, box_max);

        MatrixXd V_vol;
        V_vol.resize(points.size(), 3);
        for (int i = 0; i < points.size(); ++i) {
            const auto& v = points[i];
            V_vol.row(i) = Vector3d(v[0], v[1], v[2]);
        }

        MatrixXi T_vol;
        T_vol.resize(tets.size(), 4);
        for (int i = 0; i < tets.size(); ++i) {
            const auto& t = tets[i];
            T_vol.row(i) = Vector4i(t[0], t[1], t[2], t[3]);
        }

        MatrixXr V_emb_r;
        MatrixXi T_emb;
        MatrixXi F_on_surface;
        embed_surface(V_surface, F_surface, V_vol, T_vol, V_emb_r, T_emb, F_on_surface);

        MatrixXd V_emb;
        if (!VF_rational_to_double(V_emb_r, T_emb, V_emb)) {
            log_and_throw_error("Tets are inverted after converting to double precision.");
        }

        // add tags
        VectorXi T_tags;
        tag_tets_from_image(input_paths[0], V_emb, T_emb, T_tags);

        // write MSH
        {
            wmtk::MshData msh;
            msh.add_tet_vertices(V_emb.rows(), [&](size_t k) -> Vector3d { return V_emb.row(k); });

            msh.add_tets(T_emb.rows(), [&](size_t k) {
                std::array<size_t, 4> data{T_emb(k, 0), T_emb(k, 1), T_emb(k, 2), T_emb(k, 3)};
                return data;
            });

            msh.add_tet_attribute<1>("tag", [&T_tags](size_t i) { return T_tags[i]; });

            msh.save("debug_input_embedding.msh", true);
        }

        // write VTU
        {
            paraviewo::VTUWriter writer;
            writer.add_cell_field("tag", T_tags.cast<double>());
            writer.write_mesh("debug_input_embedding.vtu", V_emb, T_emb);
        }
    }

    // /////////
    // // Prepare Envelope and parameter for TetWild
    // /////////

    params.init(box_minmax.first, box_minmax.second);

    std::shared_ptr<Envelope> ptr_env;
    {
        std::vector<Eigen::Vector3i> tempF(f_simplified.size());
        for (size_t i = 0; i < tempF.size(); ++i) {
            tempF[i] << f_simplified[i][0], f_simplified[i][1], f_simplified[i][2];
        }
        if (use_sample_envelope) {
            ptr_env = std::make_shared<SampleEnvelope>();
        } else {
            ptr_env = std::make_shared<ExactEnvelope>();
        }
        ptr_env->init(v_simplified, tempF, 0.5);
    }

    /////////////////////////////////////////////////////

    igl::Timer timer;
    timer.start();

    // triangle insertion with volumeremesher on the simplified mesh
    std::vector<Vector3r> v_rational;
    std::vector<std::array<size_t, 3>> facets;
    std::vector<bool> is_v_on_input;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<bool> tet_face_on_input_surface;

    std::cout << "vsimp size: " << v_simplified.size() << std::endl;
    std::cout << "fsimp size: " << f_simplified.size() << std::endl;

    igl::Timer insertion_timer;
    insertion_timer.start();

    {
        ImageSimulationMesh mesh_for_insertion(params, *ptr_env, surf_mesh.m_envelope, NUM_THREADS);
        mesh_for_insertion.insertion_by_volumeremesher(
            v_simplified,
            f_simplified,
            v_rational,
            facets,
            is_v_on_input,
            tets,
            tet_face_on_input_surface);
    }

    // generate new mesh
    image_simulation::ImageSimulationMesh mesh(params, *ptr_env, surf_mesh.m_envelope, NUM_THREADS);

    mesh.init_from_Volumeremesher(v_rational, is_v_on_input, tets, tet_face_on_input_surface);

    const double insertion_time = insertion_timer.getElapsedTime();
    wmtk::logger().info("volume remesher insertion time: {}s", insertion_time);

    mesh.consolidate_mesh();
    {
        int num_parts = mesh.flood_fill();
        logger().info("flood fill parts {}", num_parts);
    }

    // add tags
    tag_tets_from_image(input_paths[0], mesh);

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
    fout << "insertion and preprocessing" << insertion_time << std::endl;
    fout.close();

    wmtk::logger().info("final max energy = {} avg = {}", max_energy, avg_energy);
    mesh.write_msh(params.output_path + "_final.msh");
    mesh.write_vtu(params.output_path + "_1.vtu");
    mesh.write_surface(params.output_path + "_surface.obj");

    wmtk::logger().info("======= finish =========");
}

} // namespace wmtk::components::image_simulation