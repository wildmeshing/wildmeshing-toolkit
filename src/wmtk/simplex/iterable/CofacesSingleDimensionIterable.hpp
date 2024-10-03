#pragma once

#include <queue>

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/RawSimplex.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/internal/VisitedArray.hpp>

namespace wmtk::simplex {

/**
 * This iterator works pretty much the same as TopDimensionCofacesIterable.
 *
 * Besides the listed iterator depths 0 to 3, this iterator requires something that I call depth 2+.
 * This is necessary, when the iterator should also stop in the intermediate phase. That happens in
 * two cases:
 * 1. TriMesh, vertex, get edges
 * 2. TetMesh, edge, get triangles
 */
class CofacesSingleDimensionIterable
{
public:
    /**
     * The IteratorPhase is only used for depth 1 and 2.
     */
    enum IteratorPhase { Forward = 0, Intermediate = 1, Backward = 2, End = 3 };

    class Iterator
    {
    public:
        Iterator(const CofacesSingleDimensionIterable& container, const Tuple& t = Tuple());
        Iterator& operator++();
        bool operator!=(const Iterator& other) const;
        Tuple& operator*();
        const Tuple& operator*() const;

    private:
        /**
         * @brief Get the d - depth primitive type.
         *
         * Example: for a TriMesh the d-1 simplex is an edge, and the d-2 simplex a vertex.
         */
        PrimitiveType pt(int64_t depth) const;
        /**
         * @brief Compute the depth from the mesh and the simplex type.
         *
         * The depth is "mesh top simplex dimension" - "simplex dimension".
         */
        int64_t depth();
        /**
         * @brief Same as `depth()` but for the coface instead of the simplex type.
         */
        int64_t coface_depth();

        /**
         * @brief Depending on the depth, the iterator must be initialized differently.
         */
        void init();

        /**
         * @brief Just return the simplex and stop.
         *
         * Example: triangle in a TriMesh.
         */
        Iterator& step_depth_0();
        /**
         * @brief There are at max two d-simplices.
         *
         * Example: edge in a TriMesh.
         * Example: triangle in a TetMesh.
         */
        Iterator& step_depth_1();
        /**
         * @brief Iterate around simplex to find all d-simplices.
         *
         * Example: vertex in a TriMesh.
         * Example: edge in a TetMesh.
         */
        Iterator& step_depth_2();
        /**
         * @brief Use breadth first search to find all d-simplices.
         *
         * Example: vertex in a TetMesh.
         */
        Iterator& step_depth_3();

        void add_neighbors_to_queue();


    private:
        const CofacesSingleDimensionIterable* m_container;

        Tuple m_t; // the tuple that iterates through the mesh
        IteratorPhase m_phase = IteratorPhase::Forward; // for depth 1 and 2 iteration

        std::queue<Tuple> m_queue; // for depth 3 iteration
        simplex::internal::VisitedArray<int64_t> m_visited; // for depth 3 iteration
        simplex::internal::VisitedArray<simplex::RawSimplex>
            m_visited_cofaces; // for depth 3 iteration
    };

public:
    CofacesSingleDimensionIterable(
        const Mesh& mesh,
        const Simplex& simplex,
        const PrimitiveType cofaces_type);

    Iterator begin() const { return Iterator(*this, m_simplex.tuple()); }
    Iterator end() const { return Iterator(*this); }

private:
    const Mesh* m_mesh;
    const Simplex m_simplex;
    const PrimitiveType m_cofaces_type;
};

} // namespace wmtk::simplex
