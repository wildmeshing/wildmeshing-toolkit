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
        Iterator(const Mesh& mesh, const Simplex& simplex, bool is_done = false);
        Iterator operator++();
        bool operator!=(const Iterator& other) const;
        Tuple operator*();
        const Tuple& operator*() const;

    private:
        void init_trimesh_vertex();
        Iterator step_trimesh_vertex();

        Iterator step_trimesh_edge();
        Iterator step_trimesh_face();

        void init_tetmesh();
        void init_tetmesh_vertex();
        void init_tetmesh_edge();

        Iterator step_tetmesh_vertex();
        Iterator step_tetmesh_edge();
        Iterator step_tetmesh_face();
        Iterator step_tetmesh_tet();

        Iterator step_edgemesh();

        Iterator step_pointmesh();

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
