//
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include "Parameters.h"
#include "Envelope.h"
namespace wmtk {

    class TetMesh {
    public:
        // Cell Tuple Navigator
        class Tuple {
            size_t vid;
            size_t eid;//todo: need edge list and face list?
            size_t fid;
            size_t tid;

            void flip_vertex(const TetMesh &m);//along edge

            void flip_edge(const TetMesh &m);//along face

            void flip_Face(const TetMesh &m);//along tet

            void flip_tetrahedron(const TetMesh &m);//todo: along face?
        };

        class VertexConnectivity {
        private:
            std::vector<size_t> m_conn_tets;
            bool m_is_removed = false;
        };

        class EdgeConnectivity {
        private:
            std::array<size_t, 2> m_indices;
            bool m_is_removed = false;
        };

        class FaceConnectivity {
        private:
            std::array<size_t, 3> m_indices;//todo: how to set the order for flip_edge()?
            bool m_is_removed = false;
        };

        class TetrahedronConnectivity {
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


        // Stores the connectivity of the mesh
        std::vector<VertexConnectivity> m_vertex_connectivity;
        std::vector<EdgeConnectivity> m_edge_connectivity;
        std::vector<FaceConnectivity> m_face_connectivity;
        std::vector<TetrahedronConnectivity> m_tetrahedron_connectivity;

        virtual ~TetMesh(){}

        //// Split the edge in the tuple
        void split(const Tuple &t);//todo: on one edge?

        // Checks if the split should be performed or not (user controlled)
        virtual bool split_precondition(const Tuple &t);//check edge condition

        // This function computes the attributes for the added simplices
        // if it returns false then the operation is undone
        virtual bool split_postcondition(const Tuple &t);//check tet condition

        //// Collapse the edge in the tuple
        void collapse(const Tuple &t);

        // Checks if the collapse should be performed or not (user controlled)
        virtual bool collapse_precondition(const Tuple &t);

        // If it returns false then the operation is undone (the tuple indexes a vertex and tet that survived)
        virtual bool collapse_postcondition(const Tuple &t);
        //todo: quality, inversion, envelope: change v1 pos before this, only need to change partial attributes

        //// Swap the edge in the tuple
        void swapping(const Tuple &t);

        // Checks if the swapping should be performed or not (user controlled)
        virtual bool swapping_precondition(const Tuple &t);

        // If it returns false then the operation is undone (the tuple indexes TODO)
        virtual bool swapping_postcondition(const Tuple &t);
        //quality, inversion

        //// Smooth in the tuple
        void smoothing(const Tuple &t);

        // Checks if the smoothing should be performed or not (user controlled)
        virtual bool smoothing_precondition(const Tuple &t);

        // If it returns false then the operation is undone (the tuple indexes TODO)
        virtual bool smoothing_postcondition(const Tuple &t);

        // Invariants that are called on all the new or modified elements after an operation is performed
        virtual bool VertexInvariant(const Tuple &t);

        virtual bool EdgeInvariant(const Tuple &t);

        virtual bool FaceInvariant(const Tuple &t);

        virtual bool TetrahedronInvariant(const Tuple &t);

        virtual void resize_attributes(size_t v, size_t e, size_t t, size_t tt);

        void compact(); // cleans up the deleted vertices or tetrahedra, and fixes the corresponding indices
    };

    class VertexAttributes {
        Vector3 m_pos;
        Vector3f m_posf;

        bool m_is_on_surface;
        bool m_is_on_boundary;
        bool m_is_on_bbox;
        bool m_is_outside;

        Scalar m_sizing_scalars;
        Scalar m_scalars;
        bool m_is_freezed;
    };

    class EdgeAttributes {
        Scalar length;
    };

    class FaceAttributes {
        Scalar tag;
    };

    class TetrahedronAttributes {
        std::array<char, 4> m_is_surface_fs;
        std::array<char, 4> m_is_bbox_fs;
        std::array<int, 4> m_opp_t_ids;
        std::array<char, 4> m_surface_tags;

        Scalar m_qualities;
        Scalar m_scalars;
        bool m_is_outside;
    };

    class TetWild : public TetMesh {
        Parameters& m_params;
        Envelope& m_envelope;

        TetWild(Parameters& _m_params, Envelope& _m_envelope): m_params(_m_params), m_envelope(_m_envelope){}

        // Stores the attributes attached to simplices
        std::vector<VertexAttributes> m_vertex_attribute;
        std::vector<EdgeAttributes> m_edge_attribute;
        std::vector<FaceAttributes> m_face_attribute;
        std::vector<TetrahedronAttributes> m_tetrahedron_attribute;

        void resize_attributes(size_t v, size_t e, size_t t, size_t tt) {
            m_vertex_attribute.resize(v);
            m_edge_attribute.resize(e);
            m_face_attribute.resize(t);
            m_tetrahedron_attribute.resize(tt);
        }

    };
}
