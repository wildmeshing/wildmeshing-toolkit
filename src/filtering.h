//
// Created by Yixin Hu on 10/12/21.
//

#ifndef WILDMESHING_TOOLKIT_FILTERING_H
#define WILDMESHING_TOOLKIT_FILTERING_H
#include "Mesh.h"

namespace wmtk {
    void extract_surface(
            //input:
            TetMesh &mesh,
            //output:
            TriangleSoup &surface);

    void remove_tetmesh_exterior(
            //input:
            const TriangleSoup &surface,
            //in&out:
            TetMesh &mesh);

    void mark_tetmesh_exterior(
            //input:
            const TriangleSoup &surface,
            TetMesh &mesh,
            //out
            std::<bool>& is_tet_outside);
}
#endif //WILDMESHING_TOOLKIT_FILTERING_H
