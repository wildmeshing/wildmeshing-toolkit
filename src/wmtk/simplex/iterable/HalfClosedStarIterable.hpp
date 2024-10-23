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
 * The half closed star is used to determine the deleted simplices in an edge collapse.
 *
 * The result is the same as computing the intersection of the open star of the vertex and the
 * closed star of the edge that is represented by the tuple.
 *
 */
class HalfClosedStarIterable
{
public:
    enum IteratorPhase { Faces, OpenStar, Link };

    class Iterator
    {
    public:
        Iterator(HalfClosedStarIterable& container, const Tuple& t = Tuple());
        Iterator& operator++();
        bool operator!=(const Iterator& other) const;
        IdSimplex operator*();
        const IdSimplex operator*() const;

    private:
        Iterator& step_tri_mesh();
        Iterator& step_tet_mesh();

        bool step_faces();

    private:
        HalfClosedStarIterable& m_container;
        const Mesh& m_mesh;

        TopDimensionCofacesIterable::Iterator m_it;
        Tuple m_t;
        int8_t m_pt = -1;
        int8_t m_face_counter = 0;
        IteratorPhase m_phase = IteratorPhase::Faces;
    };

public:
    HalfClosedStarIterable(const Mesh& mesh, const Tuple& tuple);

    Iterator begin() { return Iterator(*this, m_tuple); }
    Iterator end() { return Iterator(*this); }

private:
    const Mesh& m_mesh;
    const Tuple m_tuple;
    TopDimensionCofacesIterable m_tdc_itrbl;
    TopDimensionCofacesIterable::Iterator m_it_end;

    std::array<simplex::internal::VisitedArray<simplex::IdSimplex>, 2>
        m_visited_cofaces; // for depth 3 iteration
    std::array<simplex::internal::VisitedArray<simplex::IdSimplex>, 2>
        m_visited_link; // for depth 3 iteration
};

} // namespace wmtk::simplex