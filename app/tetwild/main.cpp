#include <TetWild.h>
#include <igl/write_triangle_mesh.h>
#include <wmtk/TetMesh.h>

//#include <catch2/catch.hpp>
#include "spdlog/common.h"

#include <igl/read_triangle_mesh.h>

using namespace wmtk;
using namespace tetwild;

int main(int argc, char** argv)
{
    using std::cout;
    using std::endl;

    auto pausee = []() {
        std::cout << "pausing..." << std::endl;
        char c;
        std::cin >> c;
        if (c == '0') exit(0);
    };

    Eigen::MatrixXd V;
    Eigen::MatrixXd F;
    std::string input_path = WMT_DATA_DIR "/37322.stl";
    igl::read_triangle_mesh(input_path, V, F);
    cout << V.rows() << " " << F.rows() << endl;

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

    tetwild::TetWild::InputSurface input_surface;
    input_surface.params.lr = 1 / 15.0;
    input_surface.init(vertices, faces);
    input_surface.remove_duplicates();
    //
    fastEnvelope::FastEnvelope envelope;
    cout << "input_surface.params.eps " << input_surface.params.eps << endl;
    envelope.init(vertices, env_faces, input_surface.params.eps);
    //
    tetwild::TetWild mesh(input_surface.params, envelope);

    auto output_faces = [&]() {
        // output surface
        {
            auto outface = mesh.get_faces_by_condition(
                [](auto& attr) { return attr.m_is_surface_fs == true; });
            Eigen::MatrixXd matV = Eigen::MatrixXd::Zero(mesh.vert_capacity(), 3);
            for (auto v : mesh.get_vertices()) {
                auto vid = v.vid(mesh);
                matV.row(vid) = mesh.m_vertex_attribute[vid].m_posf;
            }
            Eigen::MatrixXi matF(outface.size(), 3);
            for (auto i = 0; i < outface.size(); i++) {
                matF.row(i) << outface[i][0], outface[i][1], outface[i][2];
            }
            std::cout << outface.size() << std::endl;
            igl::write_triangle_mesh("track_surface.obj", matV, matF);
        }
        {
            auto outface =
                mesh.get_faces_by_condition([](auto& attr) { return attr.m_is_bbox_fs >= 0; });
            Eigen::MatrixXd matV = Eigen::MatrixXd::Zero(mesh.vert_capacity(), 3);
            for (auto v : mesh.get_vertices()) {
                auto vid = v.vid(mesh);
                matV.row(vid) = mesh.m_vertex_attribute[vid].m_posf;
            }
            Eigen::MatrixXi matF(outface.size(), 3);
            for (auto i = 0; i < outface.size(); i++) {
                matF.row(i) << outface[i][0], outface[i][1], outface[i][2];
            }
            std::cout << outface.size() << std::endl;
            igl::write_triangle_mesh("track_bbox.obj", matV, matF);
        }

//        mesh.output_mesh("improved.msh");
    };

    mesh.triangle_insertion(input_surface);
    //    mesh.check_attributes();
    //    pausee();

    mesh.mesh_improvement(0);
    cout<<"ok1"<<endl;

    output_faces();
    cout<<"ok2"<<endl;

    mesh.output_mesh("final.msh");
    cout<<"ok3"<<endl;

    // todo: refine adaptively the mesh
}