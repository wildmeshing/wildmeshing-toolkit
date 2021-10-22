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

    template<class TetMesh>
    class TupleIndex{
        void flip_vertex(const TetMesh& t);
        void flip_edge(const TetMesh& t);
        void flip_triangle(const TetMesh& t);
        void flip_tetrahedron(const TetMesh& t);
    };

    template<class VertexAttribute,class EdgeAttribute,class TriangleAttribute, class TetrahedronAttribute>
    class TetMesh{
    public:
        
        // Stores the connectivity of the mesh
        std::vector<VertexConnectivity>         m_vertex_connectivity; 
        std::vector<TetrahedronConnectivity>    m_tetrahedron_connectivity; 
        
        // Stores the attributes attached to simplices
        std::vector<VertexAttributes>         m_vertex_attribute;
        std::vector<EdgeAttributes>           m_edge_attribute;
        std::vector<TriangleAttributes>       m_triangle_attribute;
        std::vector<TetrahedronAttributes>    m_tetrahedron_attribute;
        
        // Local operations

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

        void compact(); // cleans up the deleted vertices or tetrahedra, and fixes the corresponding indices
    };

}
