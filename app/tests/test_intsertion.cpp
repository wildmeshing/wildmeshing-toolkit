//
// Created by Yixin Hu on 1/6/22.
//

#include <TetWild.h>
#include <wmtk/TetMesh.h>

#include <catch2/catch.hpp>
#include "Logger.hpp"
#include "spdlog/common.h"

#include <igl/read_triangle_mesh.h>

using namespace wmtk;
using namespace tetwild;

TEST_CASE("triangle-intersion", "[test_operation]")
{
    using std::cout;
    using std::endl;

    Eigen::MatrixXd V;
    Eigen::MatrixXd F;
    std::string input_path = "../../../app/tests/37322.stl";
    igl::read_triangle_mesh(input_path, V, F);
    cout<<V.rows()<<" "<<F.rows()<<endl;

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

    tetwild::TetWild::InputSurface input_surface(vertices, faces);
    input_surface.remove_duplicates();
    //
    fastEnvelope::FastEnvelope envelope;
    envelope.init(vertices, env_faces, input_surface.params.eps);
    //
    tetwild::TetWild mesh(input_surface.params, envelope);

    mesh.triangle_insertion(input_surface);
}