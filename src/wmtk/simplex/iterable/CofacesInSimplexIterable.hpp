#pragma once

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/internal/VisitedArray.hpp>

namespace wmtk::simplex {

/**
 * Get all cofaces that are in a specific simplex type.
 */
class CofacesInSimplexIterable
{
public:
    class Iterator
    {
    public:
        Iterator(const CofacesInSimplexIterable& container, const Tuple& t = Tuple());
        Iterator operator++();
        bool operator!=(const Iterator& other) const;
        Tuple operator*();

    private:
        const CofacesInSimplexIterable* m_container;
        Tuple m_t;
    };

public:
    CofacesInSimplexIterable(
        const Mesh& mesh,
        const Simplex& simplex,
        const PrimitiveType in_simplex_type);

    Iterator begin() const { return Iterator(*this, m_simplex.tuple()); }
    Iterator end() const { return Iterator(*this); }

private:
    const Mesh* m_mesh;
    const Simplex m_simplex;
    const PrimitiveType m_in_simplex_type;
};


} // namespace wmtk::simplex
