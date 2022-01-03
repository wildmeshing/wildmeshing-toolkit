//
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include "Mesh.h"
namespace wmtk {
    void triangle_insertion(
            //input:
            const std::vector <Vector3d> &vertices, const std::vector <std::array<int, 3>> &faces,
            //output:
            TetMesh &mesh);
}
