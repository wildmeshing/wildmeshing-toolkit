#include <TetWild.h>
#include <igl/write_triangle_mesh.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Partitioning.h>
#include <CLI/CLI.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>

//#include <catch2/catch.hpp>
#include "spdlog/common.h"

#include <igl/Timer.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
//#include <wmtk/utils/GeoUtils.h>
#include <igl/predicates/predicates.h>
#include <Tracy.hpp>
#include <sec/ShortestEdgeCollapse.h>

using namespace wmtk;
using namespace tetwild;

int main(int argc, char** argv)
{
    // 
    ZoneScopedN("tetwildmain");
    using std::cout;
    using std::endl;

    tetwild::TetWild::InputSurface input_surface;

    CLI::App app{argv[0]};
    std::string input_path = WMT_DATA_DIR "/37322.stl";
    std::string output_path = "./";
    int NUM_THREADS = 1;
    app.add_option("-i,--input", input_path, "Input mesh.");
    app.add_option("-o,--output", output_path, "Output mesh.");
    app.add_option("-j,--jobs", NUM_THREADS, "thread.");
    int max_its = 10;
    app.add_option("--max-its", max_its, "max # its");
    app.add_option("--epsr", input_surface.params.epsr, "relative eps wrt diag of bbox");
    app.add_option("--lr", input_surface.params.lr, "relative ideal edge length wrt diag of bbox");
    CLI11_PARSE(app, argc, argv);

    Eigen::MatrixXd V;
    Eigen::MatrixXd F;
    igl::read_triangle_mesh(input_path, V, F);

    Eigen::VectorXi SVI, SVJ;
    Eigen::MatrixXd temp_V = V; // for STL file
    igl::remove_duplicate_vertices(temp_V, 0, V, SVI, SVJ);
    for (int i = 0; i < F.rows(); i++)
        for (int j : {0, 1, 2}) F(i, j) = SVJ[F(i, j)];

    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }

    const Eigen::MatrixXd box_min = V.colwise().minCoeff();
    const Eigen::MatrixXd box_max = V.colwise().maxCoeff();
    const double diag = (box_max - box_min).norm();

    const double envelope_size = input_surface.params.epsr * diag;
    Eigen::VectorXi dummy;
    std::vector<size_t> modified_v;
    if (!igl::is_edge_manifold(F) || !igl::is_vertex_manifold(F, dummy)) {
        auto v1 = v;
        auto tri1 = tri;
        wmtk::separate_to_manifold(v1, tri1, v, tri, modified_v);
    }

    sec::ShortestEdgeCollapse m(v, NUM_THREADS);
    m.create_mesh(v.size(), tri, modified_v, envelope_size);
    assert(m.check_mesh_connectivity_validity());
    wmtk::logger().info("input {} simplification", input_path);
    int target_verts = 0;

    m.collapse_shortest(target_verts);
    m.consolidate_mesh();
    m.write_triangle_mesh("tetwild_input_simp.obj");

    // initiate the tetwild mesh using the original envelop
    tetwild::TetWild mesh(input_surface.params, m.m_envelope, NUM_THREADS);

    //// get the simplified input
    Eigen::MatrixXd Vsimp = Eigen::MatrixXd::Zero(m.vert_capacity(), 3);
    Eigen::MatrixXi Fsimp = Eigen::MatrixXi::Constant(m.tri_capacity(), 3, -1);
    std::vector<Vector3d> vsimp(Vsimp.rows());
    std::vector<std::array<size_t, 3>> fsimp(Fsimp.rows());
    for (auto& t : m.get_vertices()) {
        auto i = t.vid(m);
        Vsimp.row(i) = m.vertex_attrs[i].pos;
        vsimp[i] = Vsimp.row(i);
    }

    for (auto& t : m.get_faces()) {
        auto i = t.fid(m);
        auto vs = m.oriented_tri_vertices(t);
        for (int j = 0; j < 3; j++) {
            Fsimp(i, j) = vs[j].vid(m);
            fsimp[i][j] = Fsimp(i, j);
        }
    }
    input_surface.init(vsimp, fsimp);

    auto partitioned_v = partition_mesh_vertices(Fsimp, NUM_THREADS);

    std::vector<int> partition_id(partitioned_v.rows());
    for (int i = 0; i < partitioned_v.rows(); i++) {
        partition_id[i] = partitioned_v(i, 0);
    }
    input_surface.partition_id = partition_id;
    /////////////////////////////////////////////////////

    igl::Timer timer;
    timer.start();
    /////////triangle insertion with the simplified mesh
    mesh.insert_input_surface(input_surface);
    /////////mesh improvement
    mesh.mesh_improvement(max_its);
    ////winding number
    mesh.filter_outside(input_surface.vertices, input_surface.faces);
    double time = timer.getElapsedTime();
    wmtk::logger().info("total time {}s", time);

    /////////output
    auto [max_energy, avg_energy] = mesh.get_max_avg_energy();
    std::ofstream fout(output_path + ".log");
    fout << "#t: " << mesh.tet_size() << endl;
    fout << "#v: " << mesh.vertex_size() << endl;
    fout << "max_energy: " << max_energy << endl;
    fout << "avg_energy: " << avg_energy << endl;
    fout << "eps: " << input_surface.params.eps << endl;
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
        wmtk::logger().info("Output face size {}", outface.size());
        igl::write_triangle_mesh(output_path + "_surface.obj", matV, matF);
    }

    // todo: refine adaptively the mesh
    return 0;
}