//
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include "Parameters.h"
#include "Envelope.h"
namespace wmtk{
    ////connectivity
    class VertexConnectivity{
    private:
        std::vector<size_t> m_conn_tets;
        bool m_is_removed = false;
    };

    class FaceConnectivity{
    private:
    };

    class EdgeConnectivity{
    private:
    };

    class TetConnectivity{
    private:
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

    class BaseEdgeAttr{

    };

    class BaseFaceAttr{

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


class T{
        void splitting_update_tet(size_t v1_id, size_t v2_id, size_t v_id) = 0;
        void collapsing_update_tet(size_t v1_id, size_t v2_id) = 0;
        void swapping_update_tet(size_t v1_id, size_t v2_id) = 0;
        void smoothing_update_tet(size_t v_id) = 0;

}

    template<class VertexAttr,class TetAttr>
    class TetMesh{
    public:
        std::vector<Vertex> m_vertices; 
        // missing edge and faces
        std::vector<Tet> m_tets;
        
        std::vector<VertexAttr> m_v_attribute;
        // // missing edge and face
        std::vector<TetAttr> m_t_attribute;

        virtual bool splitting_check_tet(size_t v1_id, size_t v2_id, size_t v_id) = 0;
        virtual bool collapsing_check_tet(size_t v1_id, size_t v2_id) = 0;
        virtual bool swapping_check_tet(size_t v1_id, size_t v2_id) = 0;
        virtual bool smoothing_check_tet(size_t v_id) = 0;

        virtual void splitting_update_tet(size_t v1_id, size_t v2_id, size_t v_id) = 0;
        virtual void collapsing_update_tet(size_t v1_id, size_t v2_id) = 0;
        virtual void swapping_update_tet(size_t v1_id, size_t v2_id) = 0;
        virtual void smoothing_update_tet(size_t v_id) = 0;

        void splitting(size_t v1_id, size_t v2_id, size_t v_id) = 0;
        void collapsing(size_t v1_id, size_t v2_id) = 0;
        void swapping(size_t v1_id, size_t v2_id) = 0;
        void smoothing(size_t v_id) = 0;

    };

    struct TriangleSoup{
        std::vector<Vector3f> m_vertices;
        std::vector<std::array<size_t, 3>> m_faces;
        //
        std::vector<int> m_tags;
    };
}
