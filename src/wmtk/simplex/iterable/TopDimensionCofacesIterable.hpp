#pragma once

#include <queue>

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/internal/VisitedArray.hpp>

namespace wmtk::simplex {

/**
 * Iterating through the d-simplices of a mesh can be done in different ways, depending on the
 * simplex dimension x around which the iteration is done. More precisely, the type of iteration
 * depends on the depth = d - x.
 *
 * * depth 0: no iteration necessary
 * * depth 1: there are at most 2 d-simplices
 * * depth 2: circular iteration
 * * depth 3: breadth first search
 *
 * Iteration for depth 0 and 1 are straight forward.
 * Depth 3 is also rather simple as an entire breadth first search must be performed.
 *
 * Depth 2 is more complex, especially because the input simplex could be on the boundary. To reach
 * all simplices, iteration is divided in 4 phases:
 * * forward: Starting from the input simplex's tuple, use tuple switches of type d-1 and d until
 * either the iteration reaches again the input simplex, or it hits a boundary.
 * * intermediate: If the forward iteration stopped at a boundary, the iteration needs to switch to
 * the backward phase. The intermediate phase prepares that by setting the iterator to the input
 * simplex's tuple and performing a d-1 switch.
 * * backward: After successful switch, the iteration continues just like in the forward phase until
 * another boundary is hit.
 * * end: In this phase, the iteration is ended, `is_end = true`.
 */
class TopDimensionCofacesIterable
{
public:
    /**
     * The IteratorPhase is only used for depth 1 and 2.
     */
    enum IteratorPhase { Forward = 0, Intermediate = 1, Backward = 2, End = 3 };

    class Iterator
    {
    public:
        Iterator(TopDimensionCofacesIterable& container, const Tuple& t = Tuple());
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
         * @brief Depending on the depth, the iterator must be initialized differently.
         */
        void init(int64_t depth);

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
        TopDimensionCofacesIterable* m_container;

        Tuple m_t; // the tuple that iterates through the mesh
        IteratorPhase m_phase = IteratorPhase::Forward; // for depth 1 and 2 iteration
    };

public:
    TopDimensionCofacesIterable(
        const Mesh& mesh,
        const Simplex& simplex,
        const bool retrieve_intermediate_tuple = false);

    Iterator begin() { return Iterator(*this, m_simplex.tuple()); }
    Iterator end() { return Iterator(*this); }

private:
    const Mesh* m_mesh;
    const Simplex m_simplex;

    bool m_retrieve_intermediate_tuple = false;

    std::queue<Tuple> m_queue; // for depth 3 iteration
    simplex::internal::VisitedArray<int64_t> m_visited; // for depth 3 iteration
};

} // namespace wmtk::simplex
