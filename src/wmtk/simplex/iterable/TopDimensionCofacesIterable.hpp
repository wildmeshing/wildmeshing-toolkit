#pragma once

#include <queue>

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>

namespace wmtk::simplex {

/**
 * For TriMesh:
 * Iterate first in forward direction, using `switch_tuples(t, {PF, PE})`.
 * If a boundary is hit, restart from the input simplex, and switch direction.
 *
 * Stop iterating if the boundary was hit twice, or the input simplex was reached again.
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
        Iterator(const Mesh& mesh, const Simplex& simplex, bool is_end = false);
        Iterator operator++();
        bool operator!=(const Iterator& other) const;
        Tuple operator*();
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
        Iterator step_depth_0();
        /**
         * @brief There are at max two d-simplices.
         *
         * Example: edge in a TriMesh.
         * Example: triangle in a TetMesh.
         */
        Iterator step_depth_1();
        /**
         * @brief Iterate around simplex to find all d-simplices.
         *
         * Example: vertex in a TriMesh.
         * Example: edge in a TetMesh.
         */
        Iterator step_depth_2();
        /**
         * @brief Use breadth first search to find all d-simplices.
         *
         * Example: vertex in a TetMesh.
         */
        Iterator step_depth_3();


    private:
        const Mesh* m_mesh;

        Simplex m_simplex; // the input simplex
        Tuple m_t; // the tuple that iterates through the mesh
        IteratorPhase m_phase = IteratorPhase::Forward; // for depth 1 and 2 iteration
        bool m_is_end = false; // mark iterator as end

        std::queue<Tuple> m_queue; // for depth 3 iteration
        std::vector<bool> m_visited; // for depth 3 iteration
    };

public:
    TopDimensionCofacesIterable(const Mesh& mesh, const Simplex& simplex);

    Iterator begin() const { return Iterator(*m_mesh, m_simplex); }
    Iterator end() const { return Iterator(*m_mesh, m_simplex, true); }

private:
    const Mesh* m_mesh;
    Simplex m_simplex;
};

} // namespace wmtk::simplex
