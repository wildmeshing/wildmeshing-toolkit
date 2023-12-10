#include "RawSimplexCollection.hpp"

#include "SimplexCollection.hpp"

#include <algorithm>

namespace wmtk::simplex {

std::vector<RawSimplex> RawSimplexCollection::simplex_vector(const long dimension) const
{
    std::vector<RawSimplex> simplices;
    simplices.reserve(m_simplices.size() / 3); // giving the vector some (hopefully) resonable size

    // add simplices to the vector
    for (const RawSimplex& s : m_simplices) {
        if (s.dimension() == dimension) {
            simplices.emplace_back(s);
        }
    }

    return simplices;
}

void RawSimplexCollection::add(const RawSimplex& simplex)
{
    m_simplices.push_back(simplex);
}

void RawSimplexCollection::add(const RawSimplexCollection& simplex_collection)
{
    const auto& s = simplex_collection.m_simplices;
    m_simplices.insert(m_simplices.end(), s.begin(), s.end());
}

void RawSimplexCollection::add(const SimplexCollection& simplex_collection)
{
    const std::vector<Simplex>& simplices = simplex_collection.simplex_vector();

    m_simplices.reserve(m_simplices.size() + simplices.size());
    for (const Simplex& s : simplices) {
        add(RawSimplex(simplex_collection.mesh(), s));
    }
}

void RawSimplexCollection::add(
    const Mesh& mesh,
    const PrimitiveType& ptype,
    const std::vector<Tuple>& tuple_vec)
{
    m_simplices.reserve(m_simplices.size() + tuple_vec.size());

    for (const Tuple& t : tuple_vec) {
        m_simplices.emplace_back(RawSimplex(mesh, Simplex(ptype, t)));
    }
}

} // namespace wmtk::simplex
