#pragma once
#include "Mesh.h"

namespace wmtk {
    void extract_surface(
            //input:
            TetMesh &mesh,
            //output:
            TriangleSoup &surface);

    void remove_tetmesh_exterior( // what does this do?
            //input:
            const TriangleSoup &surface,
            //in&out:
            TetMesh &mesh);

    void mark_tetmesh_exterior( // what does this one do?
            //input:
            const TriangleSoup &surface,
            TetMesh &mesh,
            //out
            std::<bool>& is_tet_outside);
}
