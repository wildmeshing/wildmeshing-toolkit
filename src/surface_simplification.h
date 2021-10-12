//
// Created by Yixin Hu on 10/12/21.
//

#ifndef WILDMESHING_TOOLKIT_SURFACE_SIMPLIFICATION_H
#define WILDMESHING_TOOLKIT_SURFACE_SIMPLIFICATION_H
#include "Mesh.h"

namespace wmtk{
    void surface_simplication(
            //input:
            const TriangleSoup& input,
            std::function<bool(const TriangleSoup&, int, int)> pre_checks,
            std::function<bool(const TriangleSoup&, int, int)> post_checks,
            //output:
            TriangleSoup& output);
}
#endif //WILDMESHING_TOOLKIT_SURFACE_SIMPLIFICATION_H
