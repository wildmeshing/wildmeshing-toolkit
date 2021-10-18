//
// Created by Yixin Hu on 10/12/21.
//

#ifndef WILDMESHING_TOOLKIT_MESH_H
#define WILDMESHING_TOOLKIT_MESH_H
#include "Parameters.h"
#include "Envelope.h"
namespace wmtk{
    ////base classes
    class Vertex{
    public:
        std::vector<size_t> conn_tets;
        bool is_removed = false;
        bool is_freezed = false;
    };
    class Edge{
    public:
    };
    class Element{
    public:
        std::array<size_t, 4> indices;
        bool is_removed = false;
    };


    ////derived classes
    class TetVertex: Vertex {
    public:
        Vector3 pos;
        Vector3f posf;

        bool is_on_surface = false;
        bool is_on_boundary = false;
        bool is_on_bbox = false;
        bool is_outside = false;

        Scalar sizing_scalar = 1;
        Scalar scalar = 0;
    };

    class TetEdge: Edge{

    };

    class Tet: Element{
    public:
        double energy;

        inline size_t &operator[](const size_t index) {
            assert(index >= 0 && index < 4);
            return indices[index];
        }

        inline size_t operator[](const size_t index) const {
            assert(index >= 0 && index < 4);
            return indices[index];
        }
    };

    class TetMesh{
        Vertex* v = new Vertex();
        Element* ele = new Element();

    public:
        std::vector<TetVertex> tet_vertices;
        std::vector<Tet> tets;

        std::array<char, 4> is_surface_fs;
        std::array<char, 4> is_bbox_fs;
        std::array<int, 4> opp_t_ids;
        std::array<char, 4> surface_tags;

        Scalar quality = 0;
        Scalar scalar = 0;
        bool is_outside = false;

        Parameters& param;
        Envelope& envelope;
        TetMesh(Parameters& _param, Envelope& _envelope): param(_param), envelope(_envelope){}
    };

    struct TriangleSoup{
        std::vector<Vector3f> vertices;
        std::vector<std::array<size_t, 3>> faces;
    };
}

#endif //WILDMESHING_TOOLKIT_MESH_H
