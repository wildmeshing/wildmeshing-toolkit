//
// Created by Yixin Hu on 10/12/21.
//

#ifndef WILDMESHING_TOOLKIT_MESH_H
#define WILDMESHING_TOOLKIT_MESH_H
#include "Parameters.h"

namespace wmtk{
    struct TriangleSoup{
        std::vector<Vector3f> vertices;
        std::vector<std::array<int, 3>> faces;
    };

    class TetVertex {
    public:
        Vector3 pos;
        Vector3f posf;
        std::vector<int> conn_tets;
        bool is_removed = false;
    }

    class Tet{
    public:
        std::array<int, 4> indices;
        bool is_removed = false;
        double energy;

        inline int &operator[](const int index) {
            assert(index >= 0 && index < 4);
            return indices[index];
        }

        inline int operator[](const int index) const {
            assert(index >= 0 && index < 4);
            return indices[index];
        }
    };

    class TetMesh{
    public:
        std::vector<TetVertex> tet_vertices;
        std::vector<Tet> tets;

        Parameters& param;
        Envelope& envelope;
        TetMesh(Parameters& _param, Envelope& _envelope): param(_param), envelope(_envelope){}
    };
}

#endif //WILDMESHING_TOOLKIT_MESH_H
