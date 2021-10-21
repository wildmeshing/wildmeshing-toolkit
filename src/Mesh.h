//
// Created by Yixin Hu on 10/12/21.
//

#ifndef WILDMESHING_TOOLKIT_MESH_H
#define WILDMESHING_TOOLKIT_MESH_H
#include "Parameters.h"
#include "Envelope.h"
namespace wmtk{
    ////connectivity
    class Vertex{
    public:
        std::vector<size_t> conn_tets;
        bool is_removed = false;
    };

    class Edge{
    public:
    };

    class Tet{
    public:
        std::array<size_t, 4> indices;
        bool is_removed = false;

        inline size_t &operator[](const size_t index) {
            assert(index >= 0 && index < 4);
            return indices[index];
        }

        inline size_t operator[](const size_t index) const {
            assert(index >= 0 && index < 4);
            return indices[index];
        }
    };

    ////attributes
    class BaseVertexAttr{
    public:
        //virtual functions
        virtual ~BaseVertexAttr(){}

        virtual void splitting_check_vertex(size_t v1_id, size_t v2_id, size_t v_id) = 0;
        virtual void collapsing_check_vertex(size_t v1_id, size_t v2_id) = 0;
        virtual void swapping_check_vertex(size_t v1_id, size_t v2_id) = 0;
        virtual void smoothing_check_vertex(size_t v_id) = 0;

        virtual void splitting_update_vertex(size_t v1_id, size_t v2_id, size_t v_id) = 0;
        virtual void collapsing_update_vertex(size_t v1_id, size_t v2_id) = 0;
        virtual void swapping_update_vertex(size_t v1_id, size_t v2_id) = 0;
        virtual void smoothing_update_vertex(size_t v_id) = 0;
    };

    class BaseTetAttr{
    public:
        virtual ~BaseTetAttr(){}

        virtual void splitting_check_tet(size_t v1_id, size_t v2_id, size_t v_id) = 0;
        virtual void collapsing_check_tet(size_t v1_id, size_t v2_id) = 0;
        virtual void swapping_check_tet(size_t v1_id, size_t v2_id) = 0;
        virtual void smoothing_check_tet(size_t v_id) = 0;

        virtual void splitting_update_tet(size_t v1_id, size_t v2_id, size_t v_id) = 0;
        virtual void collapsing_update_tet(size_t v1_id, size_t v2_id) = 0;
        virtual void swapping_update_tet(size_t v1_id, size_t v2_id) = 0;
        virtual void smoothing_update_tet(size_t v_id) = 0;
    };


    ////derived attributes
    class VertexAttr: BaseVertexAttr {
    public:
        std::vector<Vertex>& vertices;
        std::vector<Tet>& tets;
        Parameters& params;
        Envelope& envelope;

        VertexAttr(std::vector<Vertex>& _vertices, std::vector<Tet>& _tets, Parameters& _param, Envelope& _envelope):
                vertices(_vertices), tets(_tets), params(_param), envelope(_envelope){}
        ~VertexAttr(){}

        std::vector<Vector3> pos;
        std::vector<Vector3f> posf;

        std::vector<bool> is_on_surface;
        std::vector<bool> is_on_boundary;
        std::vector<bool> is_on_bbox;
        std::vector<bool> is_outside;

        std::vector<Scalar> sizing_scalars;
        std::vector<Scalar> scalars;
        std::vector<bool> is_freezed;

        void splitting_check_vertex(size_t v1_id, size_t v2_id, size_t v_id) override;
        void collapsing_check_vertex(size_t v1_id, size_t v2_id) override;
        void swapping_check_vertex(size_t v1_id, size_t v2_id) override;
        void smoothing_check_vertex(size_t v_id) override;

        void splitting_update_vertex(size_t v1_id, size_t v2_id, size_t v_id) override;
        void collapsing_update_vertex(size_t v1_id, size_t v2_id) override;
        void swapping_update_vertex(size_t v1_id, size_t v2_id) override;
        void smoothing_update_vertex(size_t v_id) override;
    };

    class TetAttr: BaseTetAttr{
    public:
        std::vector<Vertex> vertices;
        std::vector<Tet> tets;
        Parameters& params;
        Envelope& envelope;

        TetAttr(std::vector<Vertex>& _vertices, std::vector<Tet>& _tets, Parameters& _param, Envelope& _envelope):
                vertices(_vertices), tets(_tets), params(_param), envelope(_envelope){}
        ~TetAttr(){}

        std::vector<std::array<char, 4>> is_surface_fs;
        std::vector<std::array<char, 4>> is_bbox_fs;
        std::vector<std::array<int, 4>> opp_t_ids;
        std::vector<std::array<char, 4>> surface_tags;

        std::vector<Scalar> qualities;
        std::vector<Scalar> scalars;
        std::vector<bool> is_outside;

        void splitting_check_tet(size_t v1_id, size_t v2_id, size_t v_id) override;
        void collapsing_check_tet(size_t v1_id, size_t v2_id) override;
        void swapping_check_tet(size_t v1_id, size_t v2_id) override;
        void smoothing_check_tet(size_t v_id) override;

        void splitting_update_tet(size_t v1_id, size_t v2_id, size_t v_id) override;
        void collapsing_update_tet(size_t v1_id, size_t v2_id) override;
        void swapping_update_tet(size_t v1_id, size_t v2_id) override;
        void smoothing_update_tet(size_t v_id) override;
    };

    class TetMesh{
    public:
        std::vector<Vertex> vertices;
        std::vector<Tet> tets;
        //
        std::unique_ptr<VertexAttr> v_attribute;
        std::unique_ptr<TetAttr> t_attribute;
    };

    struct TriangleSoup{
        std::vector<Vector3f> vertices;
        std::vector<std::array<size_t, 3>> faces;
        //
        std::vector<int> tags;
    };
}

#endif //WILDMESHING_TOOLKIT_MESH_H
