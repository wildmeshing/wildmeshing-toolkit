//
// Created by Yixin Hu on 10/12/21.
//

#ifndef WILDMESHING_TOOLKIT_VERTEX_SMOOTHING_H
#define WILDMESHING_TOOLKIT_VERTEX_SMOOTHING_H
#include "Mesh.h"

namespace wmtk {
    void vertex_smoothing(TetMesh &mesh,
                          std::function<bool(TetMesh&, int, int)> pre_checks,
                          std::function<bool(TetMesh&, int, int)> post_checks);

    bool smooth_a_vertex(TetMesh &mesh, int v_id,
            std::function<bool(TetMesh&, int, int)> pre_checks,
            std::function<bool(TetMesh&, int, int)> post_checks);
}
#endif //WILDMESHING_TOOLKIT_VERTEX_SMOOTHING_H
