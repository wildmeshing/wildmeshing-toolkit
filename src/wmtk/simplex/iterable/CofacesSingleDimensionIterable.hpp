#pragma once

#include <queue>

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/IdSimplex.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/internal/VisitedArray.hpp>

#include "TopDimensionCofacesIterable.hpp"

namespace wmtk::simplex {

/**
 * This iterator internally uses TopDimensionCofacesIterable.
 *
 * The iteration for depths 0 to 2 are the same. For depth 3, the BFS is extended to find all
 * cofaces within a single d-simplex.
 *
 * In depth 2, we need the iterator in the intermediate phase.
 */
class CofacesSingleDimensionIterable
{
public:
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
         * @brief Check if coface type is the mesh's top simplex type.
         */
        bool is_coface_d0();

        /**
         * @brief Depending on the depth, the iterator must be initialized differently.
         */
        void init();

        /**
         * Use breadth first search to find all d-simplices, and iterate through all cofaces in a
         * d-simplex.
         */
        Iterator& step_depth_3();

    private:
        CofacesSingleDimensionIterable* m_container;
        TopDimensionCofacesIterable::Iterator m_it;
        int8_t m_edge_counter = 0;
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

    simplex::internal::VisitedArray<simplex::IdSimplex> m_visited_cofaces; // for depth 3 iteration
};

} // namespace wmtk::simplex
