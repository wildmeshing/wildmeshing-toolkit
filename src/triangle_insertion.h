//
// Created by Yixin Hu on 10/12/21.
//

#ifndef WILDMESHING_TOOLKIT_TRIANGLE_INSERTION_H
#define WILDMESHING_TOOLKIT_TRIANGLE_INSERTION_H

#include "Mesh.h"
namespace wmtk {
    void triangle_insertion(
            //input:
            const std::vector <Vector3f> &vertices, const std::vector <std::array<int, 3>> &faces,
            //output:
            TetMesh &mesh);
}
#endif //WILDMESHING_TOOLKIT_TRIANGLE_INSERTION_H
