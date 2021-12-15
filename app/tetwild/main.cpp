//
// Created by Yixin Hu on 10/17/21.
//

#include "TetWild.h"

int main(int argc, char **argv)
{
    using namespace tetwild;

    Parameters params;
    params.lr = 1 / 20.;
    params.init(Vector3f(0, 0, 0), Vector3f(1, 1, 1));

    Envelope envelope;
    TetWild tetwild(params, envelope);
    //    tetwild.test();

    std::vector<VertexAttributes> vertices(4);
    vertices[0].m_posf = Vector3f(0, 0, 0);
    vertices[1].m_posf = Vector3f(1, 0, 0);
    vertices[2].m_posf = Vector3f(0, 1, 0);
    vertices[3].m_posf = Vector3f(0, 0, 1);
    std::vector<std::array<size_t, 4>> tets = {{{0, 1, 3, 2}}};
    std::vector<TetAttributes> tet_attrs(1);

    tetwild.init(vertices.size(), tets);
    tetwild.create_mesh_attributes(vertices, tet_attrs);

    tetwild.split_all_edges();

    tetwild.output_mesh(argv[1]);

    return 0;
}