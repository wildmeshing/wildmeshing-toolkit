#pragma once

#include "Mesh.h"
namespace wmtk {
    void triangle_insertion(
            //input:
            const std::vector <Vector3f> &vertices, const std::vector <std::array<int, 3>> &faces,
            //output:
            TetMesh &mesh);
}
