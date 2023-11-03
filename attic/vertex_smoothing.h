#pragma once
#include "Mesh.h"

namespace wmtk {
    void vertex_smoothing(TetMesh &mesh,
                          std::function<bool(TetMesh&, int, int)> pre_checks,
                          std::function<bool(TetMesh&, int, int)> post_checks);

    bool smooth_a_vertex(TetMesh &mesh, int v_id,
            std::function<bool(TetMesh&, int, int)> pre_checks,
            std::function<bool(TetMesh&, int, int)> post_checks);
}
