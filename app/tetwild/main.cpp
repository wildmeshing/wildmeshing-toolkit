#include <TetWild.h>
#include <igl/write_triangle_mesh.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Partitioning.h>

#include <CLI/CLI.hpp>
//#include <catch2/catch.hpp>
#include "spdlog/common.h"

#include <igl/read_triangle_mesh.h>
#include <igl/Timer.h>
//#include <wmtk/utils/GeoUtils.h>
#include <igl/predicates/predicates.h>

using namespace wmtk;
using namespace tetwild;

int main(int argc, char** argv)
{
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
    int max_its = 80;
    app.add_option("--max-its", max_its, "max # its");
    app.add_option("--epsr", input_surface.params.epsr, "relative eps wrt diag of bbox");
    app.add_option("--lr", input_surface.params.lr, "relative ideal edge length wrt diag of bbox");
    CLI11_PARSE(app, argc, argv);

    Eigen::MatrixXd V;
    Eigen::MatrixXd F;
    igl::read_triangle_mesh(input_path, V, F);
    cout << V.rows() << " " << F.rows() << endl;

    igl::Timer timer;
    timer.start();

    /////////input
    std::vector<Vector3d> vertices(V.rows());
    std::vector<std::array<size_t, 3>> faces(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        vertices[i] = V.row(i);
    }
    std::vector<fastEnvelope::Vector3i> env_faces(F.rows()); // todo: add new api for envelope
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            faces[i][j] = F(i, j);
            env_faces[i][j] = F(i, j);
        }
    }
    //    input_surface.params.lr = 1 / 15.0;
    input_surface.init(vertices, faces);
    input_surface.remove_duplicates(input_surface.params.diag_l);

    Eigen::MatrixXd new_F(input_surface.faces.size(), 3);
    for (int i = 0; i < input_surface.faces.size(); i++) {
        new_F(i, 0) = input_surface.faces[i][0];
        new_F(i, 1) = input_surface.faces[i][1];
        new_F(i, 2) = input_surface.faces[i][2];
    }
    auto partitioned_v = partition_mesh_vertices(new_F, NUM_THREADS);

    std::vector<int> partition_id(partitioned_v.rows());
    for (int i = 0; i < partitioned_v.rows(); i++) {
        partition_id[i] = partitioned_v(i, 0);
    }
    input_surface.partition_id = partition_id;
    //
    fastEnvelope::FastEnvelope envelope;
    wmtk::logger().info("input_surface.params.eps {}", input_surface.params.eps);
    envelope.init(vertices, env_faces, input_surface.params.eps);
    //
    tetwild::TetWild mesh(input_surface.params, envelope, NUM_THREADS);

    /////////triangle insertion
    mesh.triangle_insertion(input_surface);
    //    mesh.check_attributes();

    /////////mesh improvement
    mesh.mesh_improvement(max_its);
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

    //    mesh.output_faces(output_path+"surface.obj", [](auto& attr) { return attr.m_is_surface_fs == true; }); mesh.output_faces("bbox.obj", [](auto& attr) { return attr.m_is_bbox_fs >= 0; });
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