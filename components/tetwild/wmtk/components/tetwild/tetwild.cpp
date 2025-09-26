#include "tetwild.hpp"

// #include <remeshing/UniformRemeshing.h>
#include <sec/ShortestEdgeCollapse.h>
#include "Parameters.h"
#include "TetWildMesh.h"

#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Partitioning.h>
#include <wmtk/utils/Reader.hpp>

#include <memory>
#include <vector>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>
#include <wmtk/utils/partition_utils.hpp>
#include "wmtk/utils/InsertTriangleUtils.hpp"
#include "wmtk/utils/Logger.hpp"

#include <geogram/basic/process.h>
#include <geogram/mesh/mesh_io.h>
#include <igl/Timer.h>
#include <igl/boundary_facets.h>
#include <igl/predicates/predicates.h>
#include <igl/random_points_on_mesh.h>
#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <spdlog/common.h>

#include "tetwild_spec.hpp"

namespace wmtk::components::tetwild {

void tetwild(nlohmann::json json_params)
{
    // verify input and inject defaults
    {
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, tetwild_spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, tetwild_spec);
    }

    ZoneScopedN("tetwildmain");

    GEO::Process::enable_multithreading(false);

    tetwild::Parameters params;

    std::vector<std::string> input_paths = json_params["input"];
    std::string output_path = json_params["output"];
    bool skip_simplify = json_params["skip_simplify"];
    bool use_sample_envelope = json_params["use_sample_envelope"];
    int NUM_THREADS = json_params["num_threads"];
    int max_its = json_params["max_iterations"];
    bool filter_with_input = json_params["filter_with_input"];

    params.epsr = json_params["eps_rel"];
    params.lr = json_params["length_rel"];
    params.stop_energy = json_params["stop_energy"];

    params.preserve_topology = json_params["preserve_topology"];

    // logger settings
    {
        std::string log_file_name = json_params["log_file"];
        if (!log_file_name.empty()) {
            wmtk::set_file_logger(log_file_name);
        }
    }

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

    // rotate by an arbitrary angle
    // double theta = M_PI * 0.2;
    // Eigen::Matrix3d rotation_m;
    // rotation_m << 1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta);

    // box_minmax.first = Eigen::Vector3d(10000, 10000, 10000);
    // box_minmax.second = Eigen::Vector3d(-10000, -10000, -10000);
    // for (size_t i = 0; i < verts.size(); i++) {
    //     verts[i] = rotation_m * verts[i];
    //     if (verts[i][0] < box_minmax.first[0]) box_minmax.first[0] = verts[i][0];
    //     if (verts[i][1] < box_minmax.first[1]) box_minmax.first[1] = verts[i][1];
    //     if (verts[i][2] < box_minmax.first[0]) box_minmax.first[2] = verts[i][2];
    //     if (verts[i][0] > box_minmax.second[0]) box_minmax.second[0] = verts[i][0];
    //     if (verts[i][1] > box_minmax.second[1]) box_minmax.second[1] = verts[i][1];
    //     if (verts[i][2] > box_minmax.second[2]) box_minmax.second[2] = verts[i][2];
    // }

    double diag = (box_minmax.first - box_minmax.second).norm();
    const double envelope_size = params.epsr * diag;
    app::sec::ShortestEdgeCollapse surf_mesh(verts, NUM_THREADS, false);
    surf_mesh.create_mesh(verts.size(), tris, modified_nonmanifold_v, envelope_size / 2);
    assert(surf_mesh.check_mesh_connectivity_validity());

    if (skip_simplify == false) {
        wmtk::logger().info("input {} simplification", input_paths);
        surf_mesh.collapse_shortest(0);
        surf_mesh.consolidate_mesh();
        surf_mesh.write_triangle_mesh(output_path + "_simplified_input.obj");
    }

    params.output_path = output_path;

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

    wmtk::ExactEnvelope exact_envelope;
    {
        std::vector<Eigen::Vector3i> tempF(fsimp.size());
        for (auto i = 0; i < tempF.size(); i++) tempF[i] << fsimp[i][0], fsimp[i][1], fsimp[i][2];
        exact_envelope.init(vsimp, tempF, envelope_size / 2);
    }

    // initiate the tetwild mesh using the original envelop
    wmtk::Envelope* ptr_env;
    if (use_sample_envelope) {
        ptr_env = &(surf_mesh.m_envelope);
    } else {
        ptr_env = &(exact_envelope);
    }
    tetwild::TetWildMesh mesh(params, *ptr_env, surf_mesh.m_envelope, NUM_THREADS);

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
    // std::vector<vol_rem::bigrational> embedded_vertices;
    // std::vector<uint32_t> embedded_facets;
    // std::vector<uint32_t> embedded_cells;
    // std::vector<uint32_t> embedded_facets_on_input;
    std::vector<Vector3r> v_rational;
    std::vector<std::array<size_t, 3>> facets;
    std::vector<bool> is_v_on_input;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<bool> tet_face_on_input_surface;

    std::cout << "vsimp size: " << vsimp.size() << std::endl;
    std::cout << "fsimp size: " << fsimp.size() << std::endl;

    igl::Timer insertion_timer;
    insertion_timer.start();

    mesh.insertion_by_volumeremesher(
        vsimp,
        fsimp,
        v_rational,
        facets,
        is_v_on_input,
        tets,
        tet_face_on_input_surface);

    // generate new mesh
    tetwild::TetWildMesh mesh_new(params, *ptr_env, surf_mesh.m_envelope, NUM_THREADS);

    mesh_new.init_from_Volumeremesher(
        v_rational,
        facets,
        is_v_on_input,
        tets,
        tet_face_on_input_surface);

    double insertion_time = insertion_timer.getElapsedTime();


    // mesh_new.output_faces(output_path + "after_insertion_surface.obj", [](auto& f) {
    //     return f.m_is_surface_fs;
    // });


    wmtk::logger().info("volume remesher insertion time: {}s", insertion_time);

    // mesh_new.output_tetrahedralized_embedded_mesh(
    //     "tetrahedralized_embedded_mesh.txt",
    //     v_rational,
    //     facets,
    //     tets,
    //     tet_face_on_input_surface);

    // mesh_new.output_init_tetmesh("tetmesh_before_opt.txt");

    mesh_new.consolidate_mesh();

    // mesh_new.output_mesh(output_path + "after_insertion.msh");

    // mesh_new.output_faces("test_embed_output_bbox.obj", [](auto& f) {
    //     return f.m_is_bbox_fs != -1;
    // });

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

    // mesh_new.output_mesh(output_path + "after_optimization.msh");
    // mesh_new.output_faces(output_path + "after_optimization_surface.obj", [](auto& f) {
    //     return f.m_is_surface_fs;
    // });

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

    mesh_new.save_paraview(output_path, false);

    /////////output
    auto [max_energy, avg_energy] = mesh_new.get_max_avg_energy();
    wmtk::logger().info("final max energy = {} avg = {}", max_energy, avg_energy);

    mesh_new.output_mesh(output_path + "_final.msh");

    Eigen::MatrixXd matV; // all vertices
    Eigen::MatrixXi matF; // surface faces
    {
        auto outface = std::vector<std::array<size_t, 3>>();
        for (auto f : mesh_new.get_faces()) {
            auto res = mesh_new.switch_tetrahedron(f);
            if (!res.has_value()) {
                auto verts = mesh_new.get_face_vertices(f);
                std::array<size_t, 3> vids = {
                    {verts[0].vid(mesh_new), verts[1].vid(mesh_new), verts[2].vid(mesh_new)}};
                auto vs = mesh_new.oriented_tet_vertices(f);
                for (int j = 0; j < 4; j++) {
                    if (std::find(vids.begin(), vids.end(), vs[j].vid(mesh_new)) == vids.end()) {
                        auto res = igl::predicates::orient3d(
                            mesh_new.m_vertex_attribute[vids[0]].m_posf,
                            mesh_new.m_vertex_attribute[vids[1]].m_posf,
                            mesh_new.m_vertex_attribute[vids[2]].m_posf,
                            mesh_new.m_vertex_attribute[vs[j].vid(mesh_new)].m_posf);
                        if (res == igl::predicates::Orientation::NEGATIVE)
                            std::swap(vids[1], vids[2]);
                        break;
                    }
                }
                outface.emplace_back(vids);
            }
        }
        matV = Eigen::MatrixXd::Zero(mesh_new.vert_capacity(), 3);
        for (const auto& v : mesh_new.get_vertices()) {
            auto vid = v.vid(mesh_new);
            matV.row(vid) = mesh_new.m_vertex_attribute[vid].m_posf;
        }
        matF.resize(outface.size(), 3);
        for (auto i = 0; i < outface.size(); i++) {
            matF.row(i) << outface[i][0], outface[i][1], outface[i][2];
        }
        igl::write_triangle_mesh(output_path + "_surface.obj", matV, matF);

        wmtk::logger().info("Output face size {}", outface.size());
    }

    // Hausdorff
    double hausdorff_distance = 0;
    {
        Eigen::MatrixXd V(verts.size(), 3);
        for (int i = 0; i < verts.size(); ++i) {
            V.row(i) = verts[i];
        }
        Eigen::MatrixXi F(tris.size(), 3);
        for (int i = 0; i < tris.size(); ++i) {
            F.row(i) = Eigen::Vector3i(tris[i][0], tris[i][1], tris[i][2]);
        }

        // create envelope for output
        SampleEnvelope env;
        std::vector<Eigen::Vector3d> v_out;
        std::vector<Eigen::Vector3i> f_out;
        for (int i = 0; i < matV.rows(); ++i) {
            v_out.push_back(matV.row(i));
        }
        for (int i = 0; i < matF.rows(); ++i) {
            f_out.push_back(matF.row(i));
        }
        env.init(v_out, f_out, params.eps);

        const int n_samples = 10000;
        Eigen::MatrixXd B;
        Eigen::MatrixXi FI;
        Eigen::MatrixXd X;

        igl::random_points_on_mesh(n_samples, V, F, B, FI, X);

        for (int i = 0; i < X.rows(); ++i) {
            Eigen::Vector3d p = X.row(i);
            Eigen::Vector3d r;
            double d = env.nearest_point(p, r);
            hausdorff_distance = std::max(hausdorff_distance, d);
        }
        hausdorff_distance = std::sqrt(hausdorff_distance);
        logger().info("Hausdorff distance = {} | Envelope = {}", hausdorff_distance, params.eps);
        if (hausdorff_distance > params.eps) {
            logger().warn("Hausdorff distance is larger than the envelope!");
        }
    }


    const std::string report_file = json_params["report"];
    if (!report_file.empty()) {
        std::ofstream fout(report_file);
        nlohmann::json report;
        report["#t"] = mesh_new.tet_size();
        report["#v"] = mesh_new.vertex_size();
        report["max_energy"] = max_energy;
        report["avg_energy"] = avg_energy;
        report["eps"] = params.eps;
        report["threads"] = NUM_THREADS;
        report["time"] = time;
        report["hausdorff"] = hausdorff_distance;
        report["insertion_and_preprocessing"] = insertion_time;
        fout << std::setw(4) << report;
        fout.close();
    }

    wmtk::logger().info("======= finish =========");
}

} // namespace wmtk::components::tetwild