//
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include "Parameters.h"
#include "Envelope.h"
namespace wmtk{
    
    class TetMesh{
    public:
        // Cell Tuple Navigator
        class Tuple{
            size_t vid;
            size_t eid;
            size_t tid;
            size_t ttid;

            void flip_vertex(const TetMesh& m);
            void flip_edge(const TetMesh& m);
            void flip_triangle(const TetMesh& m);
            void flip_tetrahedron(const TetMesh& m);
        };

    class VertexConnectivity{
    private:
        std::vector<size_t> m_conn_tets;
        bool m_is_removed = false;
    };

    class TetrahedronConnectivity{
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
        std::vector<VertexConnectivity>         m_vertex_connectivity; 
        std::vector<TetrahedronConnectivity>    m_tetrahedron_connectivity; 
        
        // Split the edge in the tuple
        void split(const TupleIndex& t);
        
        // Checks if the split should be performed or not (user controlled)
        virtual bool split_precondition(const TupleIndex& t);

        // This function computes the attributes for the added simplices
        // if it returns false then the operation is undone
        virtual bool split_postcondition(const TupleIndex& t);

        // Collapse the edge in the tuple
        void collapse(const TupleIndex& t);
        
        // Checks if the collapse should be performed or not (user controlled)
        virtual bool collapse_precondition(const TupleIndex& t);

        // If it returns false then the operation is undone (the tuple indexes a vertex and tet that survived)
        virtual bool collapse_postcondition(const TupleIndex& t);

        // Collapse the edge in the tuple
        void swapping(const TupleIndex& t);
        
        // Checks if the swapping should be performed or not (user controlled)
        virtual bool swapping_precondition(const TupleIndex& t);

        // If it returns false then the operation is undone (the tuple indexes TODO)
        virtual bool swapping_postcondition(const TupleIndex& t);

        // Collapse the edge in the tuple
        void smoothing(const TupleIndex& t);
        
        // Checks if the smoothing should be performed or not (user controlled)
        virtual bool smoothing_precondition(const TupleIndex& t);

        // If it returns false then the operation is undone (the tuple indexes TODO)
        virtual bool smoothing_postcondition(const TupleIndex& t);

        // Invariants that are called on all the new or modified elements after an operation is performed
        virtual bool VertexInvariant(const TupleIndex& t);
        virtual bool EdgeInvariant(const TupleIndex& t);
        virtual bool TriangleInvariant(const TupleIndex& t);
        virtual bool TetrahedronInvariant(const TupleIndex& t);

        virtual void resize_attributes(size_t v, size_t e, size_t t, size_t tt);

        void compact(); // cleans up the deleted vertices or tetrahedra, and fixes the corresponding indices
    };

    class VertexAttributes{};
    class EdgeAttributes{};
    class TriangleAttributes{};
    class TetrahedronAttributes{};

    class TetWild : public TetMesh{
                // Stores the attributes attached to simplices
        std::vector<VertexAttributes>         m_vertex_attribute;
        std::vector<EdgeAttributes>           m_edge_attribute;
        std::vector<TriangleAttributes>       m_triangle_attribute;
        std::vector<TetrahedronAttributes>    m_tetrahedron_attribute;

        void resize_attributes(size_t v, size_t e, size_t t, size_t tt)
        {
            m_vertex_attribute.resize(v);
            m_edge_attribute.resize(e);
            m_triangle_attribute.resize(t);
            m_tetrahedron_attribute.resize(tt);
        }

    };
}
