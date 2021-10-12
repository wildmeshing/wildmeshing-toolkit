//
// Created by Yixin Hu on 10/12/21.
//

#ifndef WILDMESHING_TOOLKIT_EDGE_SPLITTING_H
#define WILDMESHING_TOOLKIT_EDGE_SPLITTING_H
#include "Mesh.h"

namespace wmtk {
    void edge_splitting(TetMesh &mesh,
                        std::function<bool(TetMesh&, int, int)> pre_checks,
                        std::function<bool(TetMesh&, int, int)> post_checks);

    bool split_an_edge(
            //input:
            TetMesh &mesh, int v1_id, int v2_id, //bool is_repush,
            std::function<bool(TetMesh&, int, int)> pre_checks,
            std::function<bool(TetMesh&, int, int)> post_checks,
            //output:
            std::vector<std::array<int, 2>> &new_edges);
}
#endif //WILDMESHING_TOOLKIT_EDGE_SPLITTING_H
