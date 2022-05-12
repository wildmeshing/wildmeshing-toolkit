#include <TetWild.h>
#include <igl/remove_unreferenced.h>
#include <igl/write_triangle_mesh.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Partitioning.h>
#include <CLI/CLI.hpp>
#include <type_traits>
#include <wmtk/utils/ManifoldUtils.hpp>
#include "fastenvelope/FastEnvelope.h"
#include "wmtk/utils/InsertTriangleUtils.hpp"

#include "Parameters.h"
#include "spdlog/common.h"

#include <igl/Timer.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/predicates/predicates.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <remeshing/UniformRemeshing.h>
#include <sec/ShortestEdgeCollapse.h>

using namespace wmtk;
using namespace tetwild;

int main(int argc, char** argv)
{
    //
    ZoneScopedN("tetwildmain");
    using std::cout;
    using std::endl;

    Parameters params;

    CLI::App app{argv[0]};
    std::string input_path = WMT_DATA_DIR "/37322.stl";
    std::string output_path = "./";
    bool skip_simplify = false;
    int NUM_THREADS = 1;
    app.add_option("-i,--input", input_path, "Input mesh.");
    app.add_option("-o,--output", output_path, "Output mesh.");
    app.add_option("-j,--jobs", NUM_THREADS, "thread.");
    app.add_flag("--skip-simplify", skip_simplify, "simplify_input.");
    int max_its = 10;
    app.add_option("--max-its", max_its, "max # its");
    app.add_option("-e, --epsr", params.epsr, "relative eps wrt diag of bbox");
    app.add_option("-r, --rlen", params.lr, "relative ideal edge length wrt diag of bbox");
    CLI11_PARSE(app, argc, argv);

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh(input_path, V, F);

    Eigen::VectorXi _I;
    igl::remove_unreferenced(V, F, V, F, _I);

    const Eigen::MatrixXd box_min = V.colwise().minCoeff();
    const Eigen::MatrixXd box_max = V.colwise().maxCoeff();
    double diag = (box_max - box_min).norm();

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

    { // open boundary (1-manifold) detection
        std::map<std::pair<size_t, size_t>, bool> open_bnd;
        for (auto i = 0; i < tris.size(); i++) {
            for (auto j = 0; j < 3; j++) {
                auto a = tris[i][j], b = tris[i][(j + 1) % 3];
                if (a > b) std::swap(a, b);
                auto [it, new_ele] = open_bnd.emplace(std::pair(a, b), true);
                if (!new_ele) {
                    it->second = false;
                }
            }
        }
        std::vector<std::pair<size_t, size_t>> open_bnd_vec;
        for (auto& [k, v] : open_bnd)
            if (v) open_bnd_vec.push_back(k);
        if (open_bnd_vec.size() > 0) wmtk::logger().warn("Open Boundary {} ", open_bnd_vec.size());
    }
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
    std::vector<Vector3d> vsimp(surf_mesh.vert_capacity());
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

    fastEnvelope::FastEnvelope exact_envelope;
    {
        std::vector<Eigen::Vector3i> tempF(fsimp.size());
        for (auto i = 0; i < tempF.size(); i++) tempF[i] << fsimp[i][0], fsimp[i][1], fsimp[i][2];
        exact_envelope.init(vsimp, tempF, envelope_size / 2);
    }

    // initiate the tetwild mesh using the original envelop
    tetwild::TetWild mesh(params, exact_envelope, NUM_THREADS);

    std::vector<size_t> partition_id(vsimp.size());
    {
        Eigen::MatrixXd new_F(fsimp.size(), 3);
        for (int i = 0; i < fsimp.size(); i++) {
            new_F(i, 0) = fsimp[i][0];
            new_F(i, 1) = fsimp[i][1];
            new_F(i, 2) = fsimp[i][2];
        }

        auto partitioned_v = partition_mesh_vertices(new_F, NUM_THREADS);
        for (auto i = 0; i < partition_id.size(); i++) partition_id[i] = partitioned_v[i];
    }
    /////////////////////////////////////////////////////

    igl::Timer timer;
    timer.start();
    /////////triangle insertion with the simplified mesh
    mesh.init_from_input_surface(vsimp, fsimp, partition_id);

    /////////mesh improvement
    mesh.mesh_improvement(max_its);
    ////winding number
    mesh.filter_outside(vsimp, fsimp);
    double time = timer.getElapsedTime();
    wmtk::logger().info("total time {}s", time);

    /////////output
    auto [max_energy, avg_energy] = mesh.get_max_avg_energy();
    std::ofstream fout(output_path + ".log");
    fout << "#t: " << mesh.tet_size() << endl;
    fout << "#v: " << mesh.vertex_size() << endl;
    fout << "max_energy: " << max_energy << endl;
    fout << "avg_energy: " << avg_energy << endl;
    fout << "eps: " << params.eps << endl;
    fout << "threads: " << NUM_THREADS << endl;
    fout << "time: " << time << endl;
    fout.close();

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

    // todo: refine adaptively the mesh
    return 0;
}