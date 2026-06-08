#include "triwild.hpp"

#include "Parameters.h"
#include "TriWildMesh.h"

#include <igl/write_triangle_mesh.h>
#include <jse/jse.h>
#include <wmtk/TriMesh.h>
#include <memory>
#include <vector>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/io/read_edge_mesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <igl/Timer.h>
#include <wmtk/utils/Delaunay.hpp>

#include "triwild_spec.hpp"

namespace wmtk::components::triwild {

void init_from_delaunay_box_mesh(const MatrixXd& V, const MatrixXi& E, MatrixXd& V_out, MatrixXi& F)
{
    assert(V.cols() == 2);

    // points for delaunay
    std::vector<wmtk::delaunay::Point2D> points(V.rows());
    // add points from surface
    for (int i = 0; i < V.rows(); i++) {
        for (int j = 0; j < 2; j++) {
            points[i][j] = V(i, j);
        }
    }

    // bbox
    Vector2d box_min = V.colwise().minCoeff();
    Vector2d box_max = V.colwise().maxCoeff();

    // increase bbox by 30% of diagonal length
    const double diagonal_length = (box_max - box_min).norm();
    const double delta = diagonal_length / 15.0;
    box_min -= Vector2d(delta, delta);
    box_max += Vector2d(delta, delta);

    // add corners of domain
    for (int i = 0; i < 4; i++) {
        Vector2d p;
        std::bitset<sizeof(int) * 4> a(i);
        for (int j = 0; j < 2; j++) {
            if (a.test(j)) {
                p[j] = box_max[j];
            } else {
                p[j] = box_min[j];
            }
        }
        points.push_back({{p[0], p[1]}});
    }

    const double voxel_resolution = diagonal_length / 20.0;
    std::array<int, 2> N; // number of grid points per dimension
    std::array<double, 2> h; // distance between grid points per dimension
    for (int i = 0; i < 2; i++) {
        const double D = box_max[i] - box_min[i];
        N[i] = (D / voxel_resolution) + 1;
        h[i] = D / N[i];
    }

    std::array<std::vector<double>, 2> ds;
    for (int i = 0; i < 2; i++) {
        ds[i].push_back(box_min[i]);
        for (int j = 0; j < N[i] - 1; j++) {
            ds[i].push_back(box_min[i] + h[i] * (j + 1));
        }
        ds[i].push_back(box_max[i]);
    }

    wmtk::SampleEnvelope envelope;
    // init envelope
    {
        std::vector<Vector2d> V_envelope;
        std::vector<Vector2i> E_envelope;
        for (int i = 0; i < E.rows(); i++) {
            E_envelope.push_back(Vector2i(E(i, 0), E(i, 1)));
        }
        for (int i = 0; i < V.rows(); i++) {
            V_envelope.push_back(V.row(i));
        }
        envelope.init(V_envelope, E_envelope, 0);
    }


    const double min_dis = voxel_resolution * voxel_resolution / 4;
    //    double min_dis = state.target_edge_len * state.target_edge_len;//epsilon*2
    for (int i = 0; i < ds[0].size(); i++) {
        for (int j = 0; j < ds[1].size(); j++) {
            if ((i == 0 || i == ds[0].size() - 1) && (j == 0 || j == ds[1].size() - 1)) {
                continue;
            }
            const Vector2d p(ds[0][i], ds[1][j]);

            Eigen::Vector2d n;
            const double sqd = envelope.nearest_point(p, n);

            if (sqd < min_dis) {
                continue;
            }
            points.push_back({{ds[0][i], ds[1][j]}});
        }
    }

    ///delaunay
    auto [unused_points, faces] = delaunay::delaunay2D(points);
    logger().info("After delaunay F = {}, V = {}", faces.size(), points.size());

    if (points.size() != unused_points.size()) {
        logger().warn(
            "Delaunay triangulation generated {} points, but input has {} points.",
            unused_points.size(),
            points.size());
    }

    // convert to output format
    F.resize(faces.size(), 3);
    for (int i = 0; i < faces.size(); i++) {
        F.row(i) = Eigen::Vector3i((int)faces[i][0], (int)faces[i][1], (int)faces[i][2]);
    }
    V_out.resize(points.size(), 2);
    for (int i = 0; i < points.size(); i++) {
        V_out.row(i) = Eigen::Vector2d(points[i][0], points[i][1]);
    }
}

void triwild(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;

    // verify input and inject defaults
    {
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, triwild_spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, triwild_spec);
    }
    const std::filesystem::path root =
        json_params.contains("json_input_file") ? json_params["json_input_file"] : "";

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

    triwild::Parameters params;

    std::string output_path = json_params["output"];
    bool skip_simplify = json_params["skip_simplify"];
    int NUM_THREADS = json_params["num_threads"];
    int max_its = json_params["max_iterations"];
    std::string filter_option = json_params["filter"];

    params.epsr = json_params["eps_rel"];
    params.lr = json_params["length_rel"];
    params.stop_energy = json_params["stop_energy"];

    params.preserve_topology = json_params["preserve_topology"];

    params.debug_output = json_params["DEBUG_output"];
    params.perform_sanity_checks = json_params["DEBUG_sanity_checks"];

    std::vector<MatrixXd> Vs;
    std::vector<MatrixXi> Es;
    // read input edge meshes
    for (const std::string& path : input_paths) {
        MatrixXd V;
        MatrixXi E;
        io::read_edge_mesh(path, V, E);
        logger().info("Read edge mesh {}: #V = {}, #E = {}", path, V.rows(), E.rows());
        V = V.block(0, 0, V.rows(), 2).eval(); // only keep x, y
        Vs.push_back(V);
        Es.push_back(E);
    }

    // generate Delaunay triangulation of the input vertices as the initial mesh
    MatrixXd V;
    MatrixXi F;
    {
        MatrixXd V_all;
        MatrixXi E_all;
        std::vector<Eigen::Vector2d> V_vec;
        std::vector<Eigen::Vector2i> E_vec;
        for (const auto& V : Vs) {
            for (int i = 0; i < V.rows(); i++) {
                V_vec.push_back(V.row(i));
            }
        }
        for (const auto& E : Es) {
            for (int i = 0; i < E.rows(); i++) {
                E_vec.push_back(E.row(i));
            }
        }
        V_all.resize(V_vec.size(), 2);
        for (int i = 0; i < V_vec.size(); i++) {
            V_all.row(i) = V_vec[i];
        }
        E_all.resize(E_vec.size(), 2);
        for (int i = 0; i < E_vec.size(); i++) {
            E_all.row(i) = E_vec[i];
        }
        init_from_delaunay_box_mesh(V_all, E_all, V, F);

        logger().info("Initial delaunay mesh: #V = {}, #F = {}", V.rows(), F.rows());

        MatrixXd V3(V.rows(), 3);
        V3.setZero();
        V3.block(0, 0, V.rows(), 2) = V;
        igl::write_triangle_mesh(output_path + "_initial_delaunay.obj", V3, F);
    }

    // insert edges one by one
    {
    }

    //// Old code

    // std::vector<Eigen::Vector3d> verts;
    // std::vector<std::array<size_t, 3>> tris;
    // std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax;
    // // double remove_duplicate_esp = params.epsr;
    // double remove_duplicate_esp = 2e-3;
    // std::vector<size_t> modified_nonmanifold_v;
    // wmtk::stl_to_manifold_wmtk_input(
    //     input_paths,
    //     remove_duplicate_esp,
    //     box_minmax,
    //     verts,
    //     tris,
    //     modified_nonmanifold_v);

    // {
    //     Eigen::MatrixXi F(tris.size(), 3);
    //     for (int i = 0; i < tris.size(); ++i) {
    //         F.row(i) = Eigen::Vector3i((int)tris[i][0], (int)tris[i][1], (int)tris[i][2]);
    //     }

    //     const auto ecs = compute_euler_characteristics(F);
    //     logger().info("Input euler characteristic: {}", ecs);
    // }

    // double diag = (box_minmax.first - box_minmax.second).norm();
    // const double envelope_size = params.epsr * diag;
    // shortest_edge_collapse::ShortestEdgeCollapse surf_mesh(
    //     verts,
    //     NUM_THREADS,
    //     !use_sample_envelope);
    // surf_mesh.create_mesh(verts.size(), tris, modified_nonmanifold_v, envelope_size / 2);
    // assert(surf_mesh.check_mesh_connectivity_validity());

    // if (skip_simplify == false) {
    //     wmtk::logger().info("input {} simplification", input_paths);
    //     surf_mesh.collapse_shortest(0);
    //     surf_mesh.consolidate_mesh();
    //     surf_mesh.write_triangle_mesh(output_path + "_simplified_input.obj");
    // }

    // params.output_path = output_path;

    // //// get the simplified input
    // std::vector<Eigen::Vector3d> vsimp(surf_mesh.vert_capacity());
    // std::vector<std::array<size_t, 3>> fsimp(surf_mesh.tri_capacity());
    // for (auto& t : surf_mesh.get_vertices()) {
    //     auto i = t.vid(surf_mesh);
    //     vsimp[i] = surf_mesh.vertex_attrs[i].pos;
    // }

    // for (auto& t : surf_mesh.get_faces()) {
    //     auto i = t.fid(surf_mesh);
    //     auto vs = surf_mesh.oriented_tri_vertices(t);
    //     for (int j = 0; j < 3; j++) {
    //         fsimp[i][j] = vs[j].vid(surf_mesh);
    //     }
    // }


    // // /////////
    // // // Prepare Envelope and parameter for TriWild
    // // /////////


    // params.init(box_minmax.first, box_minmax.second);
    // wmtk::remove_duplicates(vsimp, fsimp, 1e-10 * params.diag_l);

    // triwild::TriWildMesh mesh(params, surf_mesh.m_envelope, NUM_THREADS);

    // /////////////////////////////////////////////////////

    // igl::Timer timer;
    // timer.start();
    // std::vector<size_t> partition_id(vsimp.size());
    // wmtk::partition_vertex_morton(
    //     vsimp.size(),
    //     [&vsimp](auto i) { return vsimp[i]; },
    //     std::max(NUM_THREADS, 1),
    //     partition_id);


    // // triangle insertion with volumeremesher on the simplified mesh
    // std::vector<Vector3r> v_rational;
    // std::vector<std::array<size_t, 3>> facets;
    // std::vector<bool> is_v_on_input;
    // std::vector<std::array<size_t, 4>> tets;
    // std::vector<bool> tet_face_on_input_surface;

    // logger().info("simplified: #v = {}, #f = {}", vsimp.size(), fsimp.size());

    // igl::Timer insertion_timer;
    // insertion_timer.start();

    // mesh.insertion_by_volumeremesher_old(
    //     vsimp,
    //     fsimp,
    //     v_rational,
    //     facets,
    //     is_v_on_input,
    //     tets,
    //     tet_face_on_input_surface);

    // logger().info("=== finished insertion");

    // // generate new mesh
    // triwild::TriWildMesh mesh_new(params, surf_mesh.m_envelope, NUM_THREADS);

    // mesh_new.init_from_Volumeremesher(
    //     v_rational,
    //     facets,
    //     is_v_on_input,
    //     tets,
    //     tet_face_on_input_surface);

    // double insertion_time = insertion_timer.getElapsedTime();

    // wmtk::logger().info("volume remesher insertion time: {:.4}s", insertion_time);

    // mesh_new.consolidate_mesh();

    // // /////////mesh improvement
    // mesh_new.mesh_improvement(max_its);

    // bool all_rounded = true;
    // for (const auto& v : mesh_new.get_vertices()) {
    //     if (!mesh_new.m_vertex_attribute[v.vid(mesh_new)].m_is_rounded) {
    //         all_rounded = false;
    //         break;
    //     }
    // }
    // if (all_rounded) {
    //     wmtk::logger().info("All vertices are rounded");
    // } else {
    //     wmtk::logger().error("Not all vertices rounded!");
    // }

    // // apply input winding number
    // mesh_new.compute_winding_number(verts, tris);
    // // apply tracked surface winding number
    // mesh_new.compute_winding_number();
    // // apply flood fill
    // {
    //     int num_parts = mesh_new.flood_fill();
    //     logger().info("flood fill parts {}", num_parts);
    // }
    // // compute per-input winding number
    // mesh_new.compute_winding_numbers(input_paths);

    // // ////winding number
    // if (filter_option == "input") {
    //     mesh_new.filter_with_input_surface_winding_number();
    // } else if (filter_option == "tracked") {
    //     mesh_new.filter_with_tracked_surface_winding_number();
    // } else if (filter_option == "flood") {
    //     mesh_new.filter_with_flood_fill();
    // } else if (filter_option != "none") {
    //     logger().error("Unknown filter option '{}'. No filtering performed.", filter_option);
    // }
    // mesh_new.consolidate_mesh();

    // double time = timer.getElapsedTime();
    // logger().info("total time {:.4}s", time);
    // if (mesh_new.tet_size() == 0) {
    //     log_and_throw_error("Empty Output after Filter!");
    // }

    // /////////output
    // auto [max_energy, avg_energy] = mesh_new.get_max_avg_energy();
    // wmtk::logger().info("final max energy = {} avg = {}", max_energy, avg_energy);

    // const std::string report_file = json_params["report"];
    // if (!report_file.empty()) {
    //     std::ofstream fout(report_file);
    //     nlohmann::json report;
    //     report["#t"] = mesh_new.tet_size();
    //     report["#v"] = mesh_new.vertex_size();
    //     report["max_energy"] = max_energy;
    //     report["avg_energy"] = avg_energy;
    //     report["eps"] = params.eps;
    //     report["threads"] = NUM_THREADS;
    //     report["time"] = time;
    //     report["all_rounded"] = all_rounded;
    //     report["insertion_and_preprocessing"] = insertion_time;
    //     fout << std::setw(4) << report;
    //     fout.close();
    // }

    // // check metrics
    // if (json_params["throw_on_fail"]) {
    //     if (!all_rounded) {
    //         log_and_throw_error("Not all vertices rounded!");
    //     }
    //     if (max_energy > params.stop_energy) {
    //         log_and_throw_error("Max energy is too large.");
    //     }
    // }

    // mesh_new.write_vtu(output_path);
    // mesh_new.write_msh_groups(output_path + "_final.msh");

    logger().info("======= finish =========");
}

} // namespace wmtk::components::triwild