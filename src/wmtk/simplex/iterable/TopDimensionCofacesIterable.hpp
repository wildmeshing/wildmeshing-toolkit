#pragma once

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>

namespace wmtk::simplex {


class TopDimensionCofacesIterable
{
public:
    TopDimensionCofacesIterable(const Mesh& mesh, const Simplex& simplex);

    auto begin() { return m_collection.simplex_vector().begin(); }
    auto end() { return m_collection.simplex_vector().end(); }

private:
    SimplexCollection m_collection;
};

} // namespace wmtk::simplex
