//
// Created by Yixin Hu on 10/17/21.
//

#include "src/TetMesh.h"
#include "src/TetWild.h"

int main(int argc, char** argv) {
    using namespace wmtk;

    Parameters params;
    params.l = 1/20.;
    Envelope envelope;
    TetWild tetwild(params, envelope);
//    tetwild.test();

    std::vector<Vector3f> vertices = {Vector3f(0, 0, 0), Vector3f(1, 0, 0), Vector3f(0, 1, 0), Vector3f(0, 0, 1)};
    std::vector<std::array<size_t, 4>> tets = {{{0, 1, 2, 3}}};
    tetwild.create_mesh(vertices, tets);

    tetwild.split_all_edges();

    return 0;
}