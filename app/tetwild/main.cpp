#include <remeshing/UniformRemeshing.h>
#include <sec/ShortestEdgeCollapse.h>
#include "Parameters.h"
#include "TetWild.h"
#include "common.h"
#include "sec/envelope/SampleEnvelope.hpp"

#include <wmtk/TetMesh.h>
#include <wmtk/utils/Partitioning.h>
#include <fstream>
#include <wmtk/utils/ManifoldUtils.hpp>
#include <wmtk/utils/partition_utils.hpp>
#include "wmtk/utils/InsertTriangleUtils.hpp"
#include "wmtk/utils/Logger.hpp"

#include <geogram/mesh/mesh_io.h>
#include <igl/Timer.h>
#include <igl/boundary_facets.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/predicates/predicates.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/remove_unreferenced.h>
#include <igl/write_triangle_mesh.h>
#include <spdlog/common.h>
#include <CLI/CLI.hpp>

void reader(std::string input_surface, Eigen::MatrixXd& VI, Eigen::MatrixXi& FI)
{
    GEO::initialize();
    GEO::Mesh input;
    GEO::mesh_load(input_surface, input);
    VI.resize(input.vertices.nb(), 3);
    for (int i = 0; i < VI.rows(); i++)
        VI.row(i) << (input.vertices.point(i))[0], (input.vertices.point(i))[1],
            (input.vertices.point(i))[2];
    input.facets.triangulate();
    wmtk::logger().info("V {} F {}", input.vertices.nb(), input.facets.nb());
    FI.resize(input.facets.nb(), 3);
    for (int i = 0; i < FI.rows(); i++)
        FI.row(i) << input.facets.vertex(i, 0), input.facets.vertex(i, 1),
            input.facets.vertex(i, 2);
}

void boundary_detect(const Eigen::MatrixXi& tris)
{ // open boundary (1-manifold) detection. hand written for non-manifold meshes.
    std::map<std::pair<size_t, size_t>, int> open_bnd;
    for (auto i = 0; i < tris.rows(); i++) {
        for (auto j = 0; j < 3; j++) {
            size_t a = tris(i, j), b = tris(i, (j + 1) % 3);
            if (a > b) std::swap(a, b);
            auto key = std::pair(a, b);
            auto it = open_bnd.lower_bound(key);
            if (it == open_bnd.end() || it->first != key)
                open_bnd.emplace_hint(it, key, 1);
            else {
                it->second++;
            }
        }
    }
    auto cnt = 0;
    for (auto& [k, v] : open_bnd)
        if (v == 1) cnt++;
    if (cnt > 0) wmtk::logger().warn("Open Boundary {} ", cnt);
}

int main(int argc, char** argv)
{
    //
    ZoneScopedN("tetwildmain");

    tetwild::Parameters params;

    CLI::App app{argv[0]};
    std::string input_path = WMT_DATA_DIR "/37322.stl";
    std::string output_path = "./";
    bool skip_simplify = false;
    int NUM_THREADS = 1;
    int max_its = 10;

    app.add_option("-i,--input", input_path, "Input mesh.");
    app.add_option("-o,--output", output_path, "Output mesh.");
    app.add_option("-j,--jobs", NUM_THREADS, "thread.");
    app.add_flag("--skip-simplify", skip_simplify, "simplify_input.");
    app.add_option("--max-its", max_its, "max # its");
    app.add_option("-e, --epsr", params.epsr, "relative eps wrt diag of bbox");
    app.add_option("-r, --rlen", params.lr, "relative ideal edge length wrt diag of bbox");
    CLI11_PARSE(app, argc, argv);

    Eigen::MatrixXd inV, V;
    Eigen::MatrixXi inF, F;
    reader(input_path, inV, inF);

    boundary_detect(inF);
    Eigen::VectorXi _I;
    igl::remove_unreferenced(inV, inF, V, F, _I);

    const Eigen::Vector3d box_min = V.colwise().minCoeff();
    const Eigen::Vector3d box_max = V.colwise().maxCoeff();
    double diag = (box_max - box_min).norm();

    boundary_detect(F);
    {
        // using the same error tolerance as in tetwild
        Eigen::VectorXi SVI, SVJ;
        Eigen::MatrixXd temp_V = V; // for STL file
        igl::remove_duplicate_vertices(
            temp_V,
            std::min(1e-5, params.epsr / 10) * diag,
            V,
            SVI,
            SVJ);
        for (int i = 0; i < F.rows(); i++)
            for (int j : {0, 1, 2}) F(i, j) = SVJ[F(i, j)];
    }

    std::vector<Eigen::Vector3d> verts(V.rows());
    std::vector<std::array<size_t, 3>> tris(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        verts[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tris[i][j] = (size_t)F(i, j);
    }

    wmtk::logger().info("diag of the mesh: {} ", diag);
    boundary_detect(F);


    Eigen::VectorXi dummy;
    std::vector<size_t> frozen_verts;
    if (!igl::is_edge_manifold(F) || !igl::is_vertex_manifold(F, dummy)) {
        wmtk::logger().info("manifold separation...");
        auto v1 = verts;
        auto tri1 = tris;
        wmtk::separate_to_manifold(v1, tri1, verts, tris, frozen_verts);
    }

    const double envelope_size = params.epsr * diag;
    sec::ShortestEdgeCollapse surf_mesh(verts, NUM_THREADS, false);
    surf_mesh.create_mesh(verts.size(), tris, frozen_verts, envelope_size / 2);
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


    params.init(box_min, box_max);
    wmtk::remove_duplicates(vsimp, fsimp, params.diag_l);

    wmtk::ExactEnvelope exact_envelope;
    {
        std::vector<Eigen::Vector3i> tempF(fsimp.size());
        for (auto i = 0; i < tempF.size(); i++) tempF[i] << fsimp[i][0], fsimp[i][1], fsimp[i][2];
        exact_envelope.init(vsimp, tempF, envelope_size / 2);
    }

    // initiate the tetwild mesh using the original envelop
    tetwild::TetWild mesh(params, exact_envelope, NUM_THREADS);

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
    mesh.filter_outside({}, {}, true);
    double time = timer.getElapsedTime();
    wmtk::logger().info("total time {}s", time);

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