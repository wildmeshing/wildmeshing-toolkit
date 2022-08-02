#include <remeshing/UniformRemeshing.h>
#include <sec/ShortestEdgeCollapse.h>
#include "Parameters.h"
#include "TetWild.h"
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

    tetwild::Parameters params;

    CLI::App app{argv[0]};
    std::string input_path = WMT_DATA_DIR "/37322.stl";
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

    app.add_flag(
        "--filter-with-input",
        filter_with_input,
        "filter with input mesh, default is tracked surface.");
    app.add_flag(
        "--sample-envelope",
        use_sample_envelope,
        "use_sample_envelope for both simp and optim");
    CLI11_PARSE(app, argc, argv);

    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax;
    double remove_duplicate_esp = params.epsr;
    std::vector<size_t> modified_nonmanifold_v;
    wmtk::stl_to_manifold_wmtk_input(
        input_path,
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


    if (skip_simplify == false) {
        wmtk::logger().info("input {} simplification", input_path);
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


    /////////
    // Prepare Envelope and parameter for TetWild
    /////////


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
    tetwild::TetWild mesh(params, *ptr_env, NUM_THREADS);

    /////////////////////////////////////////////////////

    igl::Timer timer;
    timer.start();
    std::vector<size_t> partition_id(vsimp.size());
    wmtk::partition_vertex_morton(
        vsimp.size(),
        [&vsimp](auto i) { return vsimp[i]; },
        std::max(NUM_THREADS, 1),
        partition_id);
    /////////triangle insertion with the simplified mesh
    mesh.init_from_input_surface(vsimp, fsimp, partition_id);

    /////////mesh improvement
    mesh.mesh_improvement(max_its);
    ////winding number
    if (filter_with_input)
        mesh.filter_outside(verts, tris, true);
    else
        mesh.filter_outside({}, {}, true);
    mesh.consolidate_mesh();
    double time = timer.getElapsedTime();
    wmtk::logger().info("total time {}s", time);
    if (mesh.tet_size() == 0) {
        wmtk::logger().critical("Empty Output after Filter!");
        return 1;
    }

    /////////output
    auto [max_energy, avg_energy] = mesh.get_max_avg_energy();
    std::ofstream fout(output_path + ".log");
    fout << "#t: " << mesh.tet_size() << std::endl;
    fout << "#v: " << mesh.vertex_size() << std::endl;
    fout << "max_energy: " << max_energy << std::endl;
    fout << "avg_energy: " << avg_energy << std::endl;
    fout << "eps: " << params.eps << std::endl;
    fout << "threads: " << NUM_THREADS << std::endl;
    fout << "time: " << time << std::endl;
    fout.close();

    wmtk::logger().info("final max energy = {} avg = {}", max_energy, avg_energy);
    mesh.output_mesh(output_path + "_final.msh");

    {
        auto outface = std::vector<std::array<size_t, 3>>();
        for (auto f : mesh.get_faces()) {
            auto res = mesh.switch_tetrahedron(f);
            if (!res.has_value()) {
                auto verts = mesh.get_face_vertices(f);
                std::array<size_t, 3> vids = {
                    {verts[0].vid(mesh), verts[1].vid(mesh), verts[2].vid(mesh)}};
                auto vs = mesh.oriented_tet_vertices(f);
                for (int j = 0; j < 4; j++) {
                    if (std::find(vids.begin(), vids.end(), vs[j].vid(mesh)) == vids.end()) {
                        auto res = igl::predicates::orient3d(
                            mesh.m_vertex_attribute[vids[0]].m_posf,
                            mesh.m_vertex_attribute[vids[1]].m_posf,
                            mesh.m_vertex_attribute[vids[2]].m_posf,
                            mesh.m_vertex_attribute[vs[j].vid(mesh)].m_posf);
                        if (res == igl::predicates::Orientation::NEGATIVE)
                            std::swap(vids[1], vids[2]);
                        break;
                    }
                }
                outface.emplace_back(vids);
            }
        }
        Eigen::MatrixXd matV = Eigen::MatrixXd::Zero(mesh.vert_capacity(), 3);
        for (auto v : mesh.get_vertices()) {
            auto vid = v.vid(mesh);
            matV.row(vid) = mesh.m_vertex_attribute[vid].m_posf;
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