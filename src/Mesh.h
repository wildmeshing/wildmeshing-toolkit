//
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include "Parameters.h"
#include "Envelope.h"
namespace wmtk{
    ////connectivity
    class Vertex{
    public:
        std::vector<size_t> m_conn_tets;
        bool m_is_removed = false;
    };

    class Edge{
    public:
    };

    class Tet{
    public:
        std::array<size_t, 4> m_indices;
        bool m_is_removed = false;

        inline size_t &operator[](const size_t index) {
            assert(index >= 0 && index < 4);
            return m_indices[index];
        }

        inline size_t operator[](const size_t index) const {
            assert(index >= 0 && index < 4);
            return m_indices[index];
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
        std::vector<Vertex>& m_vertices;
        std::vector<Tet>& m_tets;
        Parameters& m_params;
        Envelope& m_envelope;

        VertexAttr(std::vector<Vertex>& _vertices, std::vector<Tet>& _tets, Parameters& _param, Envelope& _envelope):
                m_vertices(_vertices), m_tets(_tets), m_params(_param), m_envelope(_envelope){}
        ~VertexAttr(){}

        std::vector<Vector3> m_pos;
        std::vector<Vector3f> m_posf;

        std::vector<bool> m_is_on_surface;
        std::vector<bool> m_is_on_boundary;
        std::vector<bool> m_is_on_bbox;
        std::vector<bool> m_is_outside;

        std::vector<Scalar> m_sizing_scalars;
        std::vector<Scalar> m_scalars;
        std::vector<bool> m_is_freezed;

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
        std::vector<Vertex> m_vertices;
        std::vector<Tet> m_tets;
        Parameters& m_params;
        Envelope& m_envelope;

        TetAttr(std::vector<Vertex>& _vertices, std::vector<Tet>& _tets, Parameters& _param, Envelope& _envelope):
                m_vertices(_vertices), m_tets(_tets), m_params(_param), m_envelope(_envelope){}
        ~TetAttr(){}

        std::vector<std::array<char, 4>> m_is_surface_fs;
        std::vector<std::array<char, 4>> m_is_bbox_fs;
        std::vector<std::array<int, 4>> m_opp_t_ids;
        std::vector<std::array<char, 4>> m_surface_tags;

        std::vector<Scalar> m_qualities;
        std::vector<Scalar> m_scalars;
        std::vector<bool> m_is_outside;

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
        std::vector<Vertex> m_vertices;
        std::vector<Tet> m_tets;
        //
        std::unique_ptr<VertexAttr> m_v_attribute;
        std::unique_ptr<TetAttr> m_t_attribute;
    };

    struct TriangleSoup{
        std::vector<Vector3f> m_vertices;
        std::vector<std::array<size_t, 3>> m_faces;
        //
        std::vector<int> m_tags;
    };
}
