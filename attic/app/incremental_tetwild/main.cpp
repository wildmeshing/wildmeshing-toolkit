// #include <remeshing/UniformRemeshing.h>
#include <sec/ShortestEdgeCollapse.h>
#include "IncrementalTetWild.h"
#include "Parameters.h"
#include "common.h"
#include "sec/envelope/SampleEnvelope.hpp"

#include <wmtk/TetMesh.h>
#include <wmtk/utils/Partitioning.h>
#include <wmtk/utils/Reader.hpp>

#include <memory>
#include <vector>
#include <wmtk/utils/ManifoldUtils.hpp>
#include <wmtk/utils/partition_utils.hpp>
#include "wmtk/utils/InsertTriangleUtils.hpp"
#include "wmtk/utils/Logger.hpp"

#include <geogram/basic/process.h>
#include <geogram/mesh/mesh_io.h>
#include <igl/Timer.h>
#include <igl/boundary_facets.h>
#include <igl/predicates/predicates.h>
#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include <spdlog/common.h>
#include <CLI/CLI.hpp>


int main(int argc, char** argv)
{
    ZoneScopedN("tetwildmain");

    GEO::Process::enable_multithreading(false);

    tetwild::Parameters params;

    CLI::App app{argv[0]};
    std::string input_path = WMTK_DATA_DIR "/37322.stl";
    std::string output_path = "./";
    bool skip_simplify = false;
    bool use_sample_envelope = false;
    int NUM_THREADS = 0;
    int max_its = 10;
    bool filter_with_input = false;

    app.add_option("-i,--input", input_path, "Input mesh.");
    app.add_option("-o,--output", output_path, "Output mesh.");
    app.add_option("-j,--jobs", NUM_THREADS, "thread.");
    app.add_flag("--skip-simplify", skip_simplify, "simplify_input.");
    app.add_option("--max-its", max_its, "max # its");
    app.add_option("-e, --epsr", params.epsr, "relative eps wrt diag of bbox");
    app.add_option("-r, --rlen", params.lr, "relative ideal edge length wrt diag of bbox");
    app.add_option("--stop-energy", params.stop_energy, "stop max energy");


    app.add_flag(
        "--filter-with-input",
        filter_with_input,
        "filter with input mesh, default is tracked surface.");
    app.add_flag(
        "--sample-envelope",
        use_sample_envelope,
        "use_sample_envelope for both simp and optim");
    app.add_flag(
        "--preserve-global-topology",
        params.preserve_global_topology,
        "preserve the global topology");
    app.add_flag("--preserve-geometry", params.preserve_geometry, "preserve geometry");

    CLI11_PARSE(app, argc, argv);


    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax;
    // double remove_duplicate_esp = params.epsr;
    double remove_duplicate_esp = 2e-3;
    std::vector<size_t> modified_nonmanifold_v;
    wmtk::stl_to_manifold_wmtk_input(
        input_path,
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

    if (params.preserve_global_topology) {
        skip_simplify = true;
    }
    if (params.preserve_geometry) {
        skip_simplify = true;
    }
    if (skip_simplify == false) {
        wmtk::logger().info("input {} simplification", input_path);
        surf_mesh.collapse_shortest(0);
        surf_mesh.consolidate_mesh();
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
    wmtk::remove_duplicates(vsimp, fsimp, params.diag_l);

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
    tetwild::TetWild mesh(params, *ptr_env, surf_mesh.m_envelope, NUM_THREADS);

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
    std::vector<tetwild::Vector3r> v_rational;
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

    std::cout << "here" << std::endl;

    // generate new mesh
    tetwild::TetWild mesh_new(params, *ptr_env, surf_mesh.m_envelope, NUM_THREADS);

    if (params.preserve_geometry) {
        std::cout << "compute coplanar triangle collections start" << std::endl;
        mesh_new.detect_coplanar_triangle_collections(vsimp, fsimp);
        std::cout << "#collections: "
                  << mesh_new.triangle_collections_from_input_surface.collections.size()
                  << std::endl;
        std::cout << "compute coplanar triangle collections end" << std::endl;
    }


    mesh_new.init_from_Volumeremesher(
        v_rational,
        facets,
        is_v_on_input,
        tets,
        tet_face_on_input_surface);
    // exit(0);

    double insertion_time = insertion_timer.getElapsedTime();


    mesh_new.output_faces(output_path + "after_insertion_surface.obj", [](auto& f) {
        return f.m_is_surface_fs;
    });


    wmtk::logger().info("volume remesher insertion time: {}s", insertion_time);

    mesh_new.output_tetrahedralized_embedded_mesh(
        "tetrahedralized_embedded_mesh.txt",
        v_rational,
        facets,
        tets,
        tet_face_on_input_surface);

    mesh_new.output_init_tetmesh("tetmesh_before_opt.txt");

    std::cout << "here2" << std::endl;

    mesh_new.consolidate_mesh();

    mesh_new.output_mesh(output_path + "after_insertion.msh");
    std::cout << "here3" << std::endl;


    mesh_new.output_faces(output_path + "matched_surface.obj", [](auto& f) {
        return f.from_input_collection_id > -1;
    });

    // mesh_new.output_faces("test_embed_output_bbox.obj", [](auto& f) {
    //     return f.m_is_bbox_fs != -1;
    // });

    std::cout << "here4" << std::endl;

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

    // exit(0);
    // // test with a fixed mesh tet.obj
    // tetwild::TetWild mesh_tet(params, *ptr_env, NUM_THREADS);
    // mesh_tet.init_from_file("input_tetmesh_before_opt.txt");

    // mesh_tet.output_faces("fixed_embed_output_surface.obj", [](auto& f) {
    //     return f.m_is_surface_fs;
    // });

    // mesh_tet.output_faces("fixed_embed_output_bbox.obj", [](auto& f) {
    //     return f.m_is_bbox_fs != -1;
    // });
    // // /////////mesh improvement
    // mesh_tet.mesh_improvement(max_its);
    // std::cout << "here6" << std::endl;
    // // ////winding number
    // if (filter_with_input)
    //     mesh_tet.filter_outside(verts, tris, true);
    // else
    //     mesh_tet.filter_outside({}, {}, true);
    // mesh_tet.consolidate_mesh();
    // double time = timer.getElapsedTime();
    // wmtk::logger().info("total time {}s", time);
    // if (mesh_tet.tet_size() == 0) {
    //     wmtk::logger().critical("Empty Output after Filter!");
    //     return 1;
    // }

    // /////////output
    // auto [max_energy, avg_energy] = mesh_tet.get_max_avg_energy();
    // std::ofstream fout(output_path + ".log");
    // fout << "#t: " << mesh_tet.tet_size() << std::endl;
    // fout << "#v: " << mesh_tet.vertex_size() << std::endl;
    // fout << "max_energy: " << max_energy << std::endl;
    // fout << "avg_energy: " << avg_energy << std::endl;
    // fout << "eps: " << params.eps << std::endl;
    // fout << "threads: " << NUM_THREADS << std::endl;
    // fout << "time: " << time << std::endl;
    // fout.close();

    // wmtk::logger().info("final max energy = {} avg = {}", max_energy, avg_energy);
    // mesh_tet.output_mesh(output_path + "_final.msh");

    // {
    //     auto outface = std::vector<std::array<size_t, 3>>();
    //     for (auto f : mesh_tet.get_faces()) {
    //         auto res = mesh_tet.switch_tetrahedron(f);
    //         if (!res.has_value()) {
    //             auto verts = mesh_tet.get_face_vertices(f);
    //             std::array<size_t, 3> vids = {
    //                 {verts[0].vid(mesh_tet), verts[1].vid(mesh_tet), verts[2].vid(mesh_tet)}};
    //             auto vs = mesh_tet.oriented_tet_vertices(f);
    //             for (int j = 0; j < 4; j++) {
    //                 if (std::find(vids.begin(), vids.end(), vs[j].vid(mesh_tet)) == vids.end()) {
    //                     auto res = igl::predicates::orient3d(
    //                         mesh_tet.m_vertex_attribute[vids[0]].m_posf,
    //                         mesh_tet.m_vertex_attribute[vids[1]].m_posf,
    //                         mesh_tet.m_vertex_attribute[vids[2]].m_posf,
    //                         mesh_tet.m_vertex_attribute[vs[j].vid(mesh_tet)].m_posf);
    //                     if (res == igl::predicates::Orientation::NEGATIVE)
    //                         std::swap(vids[1], vids[2]);
    //                     break;
    //                 }
    //             }
    //             outface.emplace_back(vids);
    //         }
    //     }
    //     Eigen::MatrixXd matV = Eigen::MatrixXd::Zero(mesh_tet.vert_capacity(), 3);
    //     for (auto v : mesh_tet.get_vertices()) {
    //         auto vid = v.vid(mesh_tet);
    //         matV.row(vid) = mesh_tet.m_vertex_attribute[vid].m_posf;
    //     }
    //     Eigen::MatrixXi matF(outface.size(), 3);
    //     for (auto i = 0; i < outface.size(); i++) {
    //         matF.row(i) << outface[i][0], outface[i][1], outface[i][2];
    //     }
    //     igl::write_triangle_mesh(output_path + "_surface.obj", matV, matF);
    //     wmtk::logger().info("Output face size {}", outface.size());
    //     wmtk::logger().info("======= finish =========");
    // }

    // return 0;

    // /////////mesh improvement
    mesh_new.mesh_improvement(max_its);

    mesh_new.save_paraview(output_path, false);


    mesh_new.output_mesh(output_path + "after_optimization.msh");
    mesh_new.output_faces(output_path + "after_optimization_surface.obj", [](auto& f) {
        return f.m_is_surface_fs;
    });

    std::cout << "here6" << std::endl;
    // ////winding number
    if (filter_with_input)
        mesh_new.filter_outside(verts, tris, true);
    else
        mesh_new.filter_outside({}, {}, true);
    mesh_new.consolidate_mesh();
    double time = timer.getElapsedTime();
    wmtk::logger().info("total time {}s", time);
    if (mesh_new.tet_size() == 0) {
        wmtk::logger().critical("Empty Output after Filter!");
        return 1;
    }

    /////////output
    auto [max_energy, avg_energy] = mesh_new.get_max_avg_energy();
    std::ofstream fout(output_path + ".log");
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
    mesh_new.output_mesh(output_path + "_final.msh");

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
        Eigen::MatrixXd matV = Eigen::MatrixXd::Zero(mesh_new.vert_capacity(), 3);
        for (auto v : mesh_new.get_vertices()) {
            auto vid = v.vid(mesh_new);
            matV.row(vid) = mesh_new.m_vertex_attribute[vid].m_posf;
        }
        Eigen::MatrixXi matF(outface.size(), 3);
        for (auto i = 0; i < outface.size(); i++) {
            matF.row(i) << outface[i][0], outface[i][1], outface[i][2];
        }
        igl::write_triangle_mesh(output_path + "_surface.obj", matV, matF);

        wmtk::logger().info("Output face size {}", outface.size());
        wmtk::logger().info("======= finish =========");
    }

    return 0;
}