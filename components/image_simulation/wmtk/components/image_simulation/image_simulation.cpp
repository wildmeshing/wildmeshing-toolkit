#include "image_simulation.hpp"


#include <memory>
#include <vector>

#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Partitioning.h>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/utils/InsertTriangleUtils.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>
#include <wmtk/utils/Reader.hpp>
#include <wmtk/utils/partition_utils.hpp>

#include <sec/ShortestEdgeCollapse.h>

#include "ImageSimulationMesh.h"
#include "Parameters.h"

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

    params.preserve_topology = json_params["preserve_topology"];

    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax;
    // double remove_duplicate_esp = params.epsr;
    double remove_duplicate_esp = 2e-3;
    std::vector<size_t> modified_nonmanifold_v;
    wmtk::stl_to_manifold_wmtk_input(
        input_paths,
        remove_duplicate_esp,
        box_minmax,
        verts,
        tris,
        modified_nonmanifold_v);

    double diag = (box_minmax.first - box_minmax.second).norm();
    const double envelope_size = params.epsr * diag;
    app::sec::ShortestEdgeCollapse surf_mesh(verts, NUM_THREADS, false);
    surf_mesh.create_mesh(verts.size(), tris, modified_nonmanifold_v, envelope_size / 2);
    assert(surf_mesh.check_mesh_connectivity_validity());

    if (!skip_simplify) {
        wmtk::logger().info("input {} simplification", input_paths);
        surf_mesh.collapse_shortest(0);
        surf_mesh.consolidate_mesh();
    }

    //// get the simplified input
    std::vector<Eigen::Vector3d> vsimp(surf_mesh.vert_capacity());
    std::vector<std::array<size_t, 3>> fsimp(surf_mesh.tri_capacity());
    for (auto& t : surf_mesh.get_vertices()) {
        auto i = t.vid(surf_mesh);
        vsimp[i] = surf_mesh.vertex_attrs[i].pos;
    }

    for (auto& t : surf_mesh.get_faces()) {
        auto i = t.fid(surf_mesh);
        auto vs = surf_mesh.oriented_tri_vertices(t);
        for (int j = 0; j < 3; j++) {
            fsimp[i][j] = vs[j].vid(surf_mesh);
        }
    }


    // /////////
    // // Prepare Envelope and parameter for TetWild
    // /////////

    params.init(box_minmax.first, box_minmax.second);
    wmtk::remove_duplicates(vsimp, fsimp, 1e-10 * params.diag_l);

    // initiate the image_simulation mesh using the original envelope
    wmtk::Envelope* ptr_env;
    wmtk::ExactEnvelope exact_envelope;
    if (use_sample_envelope) {
        ptr_env = &(surf_mesh.m_envelope);
    } else {
        std::vector<Eigen::Vector3i> tempF(fsimp.size());
        for (auto i = 0; i < tempF.size(); i++) {
            tempF[i] << fsimp[i][0], fsimp[i][1], fsimp[i][2];
        }
        exact_envelope.init(vsimp, tempF, envelope_size / 2);
        ptr_env = &(exact_envelope);
    }

    /////////////////////////////////////////////////////

    igl::Timer timer;
    timer.start();
    std::vector<size_t> partition_id(vsimp.size());
    wmtk::partition_vertex_morton(
        vsimp.size(),
        [&vsimp](auto i) { return vsimp[i]; },
        std::max(NUM_THREADS, 1),
        partition_id);


    // triangle insertion with volumeremesher on the simplified mesh
    std::vector<Vector3r> v_rational;
    std::vector<std::array<size_t, 3>> facets;
    std::vector<bool> is_v_on_input;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<bool> tet_face_on_input_surface;

    std::cout << "vsimp size: " << vsimp.size() << std::endl;
    std::cout << "fsimp size: " << fsimp.size() << std::endl;

    igl::Timer insertion_timer;
    insertion_timer.start();

    {
        ImageSimulationMesh mesh(params, *ptr_env, surf_mesh.m_envelope, NUM_THREADS);
        mesh.insertion_by_volumeremesher(
            vsimp,
            fsimp,
            v_rational,
            facets,
            is_v_on_input,
            tets,
            tet_face_on_input_surface);
    }

    // generate new mesh
    image_simulation::ImageSimulationMesh mesh_new(
        params,
        *ptr_env,
        surf_mesh.m_envelope,
        NUM_THREADS);

    mesh_new.init_from_Volumeremesher(
        v_rational,
        facets,
        is_v_on_input,
        tets,
        tet_face_on_input_surface);

    double insertion_time = insertion_timer.getElapsedTime();

    wmtk::logger().info("volume remesher insertion time: {}s", insertion_time);

    mesh_new.consolidate_mesh();

    size_t nonmani_ver_cnt = 0;
    size_t surface_v_cnt = 0;
    for (auto v : mesh_new.get_vertices()) {
        if (mesh_new.m_vertex_attribute[v.vid(mesh_new)].m_is_on_surface) {
            surface_v_cnt++;
            if (mesh_new.count_vertex_links(v) > 1) {
                nonmani_ver_cnt++;
            }
        }
    }

    wmtk::logger().info("MESH NONMANIFOLD VERTEX COUNT BEFORE OPTIMIZE: {}", nonmani_ver_cnt);
    wmtk::logger().info("MESH surface VERTEX COUNT BEFORE OPTIMIZE: {}", surface_v_cnt);

    // /////////mesh improvement
    mesh_new.mesh_improvement(max_its);

    // ////winding number
    if (filter_with_input)
        mesh_new.filter_outside(verts, tris, true);
    else
        mesh_new.filter_outside({}, {}, true);
    mesh_new.consolidate_mesh();
    double time = timer.getElapsedTime();
    wmtk::logger().info("total time {}s", time);
    if (mesh_new.tet_size() == 0) {
        log_and_throw_error("Empty Output after Filter!");
    }

    mesh_new.save_paraview(params.output_path, false);

    /////////output
    auto [max_energy, avg_energy] = mesh_new.get_max_avg_energy();
    std::ofstream fout(params.output_path + ".log");
    fout << "#t: " << mesh_new.tet_size() << std::endl;
    fout << "#v: " << mesh_new.vertex_size() << std::endl;
    fout << "max_energy: " << max_energy << std::endl;
    fout << "avg_energy: " << avg_energy << std::endl;
    fout << "eps: " << params.eps << std::endl;
    fout << "threads: " << NUM_THREADS << std::endl;
    fout << "time: " << time << std::endl;
    fout << "insertion and preprocessing" << insertion_time << std::endl;
    fout.close();

    wmtk::logger().info("final max energy = {} avg = {}", max_energy, avg_energy);
    mesh_new.output_mesh(params.output_path + "_final.msh");

    {
        auto posf = [&mesh_new](const size_t i) { return mesh_new.m_vertex_attribute[i].m_posf; };

        std::vector<std::array<size_t, 3>> outface;
        for (const Tuple& f : mesh_new.get_faces()) {
            if (f.switch_tetrahedron(mesh_new)) {
                continue;
            }
            const auto verts = mesh_new.get_face_vertices(f);
            std::array<size_t, 3> vids = {
                {verts[0].vid(mesh_new), verts[1].vid(mesh_new), verts[2].vid(mesh_new)}};
            const auto vs = mesh_new.oriented_tet_vertices(f);
            for (int j = 0; j < 4; j++) {
                if (std::find(vids.begin(), vids.end(), vs[j].vid(mesh_new)) == vids.end()) {
                    auto res = igl::predicates::orient3d(
                        posf(vids[0]),
                        posf(vids[1]),
                        posf(vids[2]),
                        posf(vs[j].vid(mesh_new)));
                    if (res == igl::predicates::Orientation::NEGATIVE) {
                        std::swap(vids[1], vids[2]);
                    }
                    break;
                }
            }
            outface.emplace_back(vids);
        }
        Eigen::MatrixXd matV = Eigen::MatrixXd::Zero(mesh_new.vert_capacity(), 3);
        for (const Tuple& v : mesh_new.get_vertices()) {
            auto vid = v.vid(mesh_new);
            matV.row(vid) = posf(vid);
        }
        Eigen::MatrixXi matF(outface.size(), 3);
        for (size_t i = 0; i < outface.size(); i++) {
            matF.row(i) << outface[i][0], outface[i][1], outface[i][2];
        }
        igl::write_triangle_mesh(params.output_path + "_surface.obj", matV, matF);

        wmtk::logger().info("Output face size {}", outface.size());
        wmtk::logger().info("======= finish =========");
    }
}

} // namespace wmtk::components::image_simulation