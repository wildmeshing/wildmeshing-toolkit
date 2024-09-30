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
         * E.g., for a triangle mesh the d-1 simplex is an edge, and the d-2 simplex a vertex.
         */
        PrimitiveType pt(int64_t depth) const;
        int64_t depth();

        void init(int64_t depth);

        Iterator step_depth_0();
        Iterator step_depth_1();
        Iterator step_depth_2();
        Iterator step_depth_3();


    private:
        const Mesh* m_mesh;

        Simplex m_simplex;
        Tuple m_t;
        IteratorPhase m_phase = IteratorPhase::Forward;
        bool m_is_end = false;

        std::queue<Tuple> m_queue;
        std::vector<bool> m_visited;
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
