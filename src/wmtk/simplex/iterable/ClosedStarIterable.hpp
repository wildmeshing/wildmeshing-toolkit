#pragma once

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/IdSimplex.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/internal/VisitedArray.hpp>

#include "TopDimensionCofacesIterable.hpp"

namespace wmtk::simplex {

/**
 * This iterator internally uses TopDimensionCofacesIterable.
 *
 */
class ClosedStarIterable
{
public:
    enum IteratorPhase { Faces, OpenStar, Link };

    class Iterator
    {
    public:
        Iterator(ClosedStarIterable& container, const Tuple& t = Tuple());
        Iterator& operator++();
        bool operator!=(const Iterator& other) const;
        IdSimplex operator*();
        const IdSimplex operator*() const;

    private:
        /**
         * @brief Compute the depth from the mesh and the simplex type.
         *
         * The depth is "mesh top simplex dimension" - "simplex dimension".
         */
        int64_t depth();

        /**
         * @brief Depending on the depth, the iterator must be initialized differently.
         */
        void init();

        /**
         * Use breadth first search to find all d-simplices, and iterate through all cofaces in a
         * d-simplex.
         */
        Iterator& step_depth_3();

        Tuple navigate_to_link(Tuple t);

        void step_faces();

    private:
        ClosedStarIterable* m_container;
        TopDimensionCofacesIterable::Iterator m_it;
        Tuple m_t;
        int8_t m_pt = -1;
        int8_t m_sub_pt = -1;
        int8_t m_face_counter = 0;
        int8_t m_sub_face_counter = 0;
        IteratorPhase m_phase = IteratorPhase::Faces;
    };

public:
    ClosedStarIterable(const Mesh& mesh, const Simplex& simplex);

    Iterator begin() { return Iterator(*this, m_simplex.tuple()); }
    Iterator end() { return Iterator(*this); }

private:
    const Mesh* m_mesh;
    const Simplex m_simplex;
    TopDimensionCofacesIterable m_tdc_itrbl;
    TopDimensionCofacesIterable::Iterator m_it_end;

    std::array<simplex::internal::VisitedArray<simplex::IdSimplex>, 2>
        m_visited_cofaces; // for depth 3 iteration
    std::array<simplex::internal::VisitedArray<simplex::IdSimplex>, 2>
        m_visited_link; // for depth 3 iteration
};

} // namespace wmtk::simplex