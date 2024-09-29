#pragma once

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
        Iterator(Mesh& mesh, const Simplex& simplex, bool is_done = false);
        Iterator operator++();
        bool operator!=(const Iterator& other) const;
        Tuple operator*();
        const Tuple& operator*() const;

    private:
        void init_trimesh_vertex();
        Iterator step_trimesh_vertex();

        Iterator step_trimesh_edge();
        Iterator step_trimesh_face();

    private:
        Mesh* m_mesh;
        Simplex m_simplex;
        Tuple t;
        IteratorPhase m_phase = IteratorPhase::Forward;
        bool m_is_end = false;
    };

public:
    TopDimensionCofacesIterable(Mesh& mesh, const Simplex& simplex);

    Iterator begin() const { return Iterator(*m_mesh, m_simplex); }
    Iterator end() const { return Iterator(*m_mesh, m_simplex, true); }

private:
    Mesh* m_mesh;
    Simplex m_simplex;
};

} // namespace wmtk::simplex
