#pragma once

#include <queue>

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/RawSimplex.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/internal/VisitedArray.hpp>

#include "TopDimensionCofacesIterable.hpp"

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
        Iterator(CofacesSingleDimensionIterable& container, const Tuple& t = Tuple());
        Iterator& operator++();
        bool operator!=(const Iterator& other) const;
        Tuple& operator*();
        const Tuple& operator*() const;

    private:
        /**
         * @brief Compute the depth from the mesh and the simplex type.
         *
         * The depth is "mesh top simplex dimension" - "simplex dimension".
         */
        int64_t depth();
        /**
         * @brief Same as `depth()` but for the coface instead of the simplex type.
         */
        bool is_coface_d0();

        /**
         * @brief Depending on the depth, the iterator must be initialized differently.
         */
        void init();

        /**
         * @brief Use breadth first search to find all d-simplices.
         *
         * Example: vertex in a TetMesh.
         */
        Iterator& step_depth_3();

    private:
        CofacesSingleDimensionIterable* m_container;
        TopDimensionCofacesIterable::Iterator m_it;
    };

public:
    CofacesSingleDimensionIterable(
        const Mesh& mesh,
        const Simplex& simplex,
        const PrimitiveType cofaces_type);

    Iterator begin() { return Iterator(*this, m_simplex.tuple()); }
    Iterator end() { return Iterator(*this); }

private:
    const Mesh* m_mesh;
    const Simplex m_simplex;
    const PrimitiveType m_cofaces_type;
    TopDimensionCofacesIterable m_tdc_itrbl;
    TopDimensionCofacesIterable::Iterator m_it_end;

    simplex::internal::VisitedArray<simplex::RawSimplex> m_visited_cofaces; // for depth 3 iteration
};

} // namespace wmtk::simplex
