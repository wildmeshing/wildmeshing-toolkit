#include "RawSimplexCollection.hpp"

#include "SimplexCollection.hpp"

#include <algorithm>

namespace wmtk::simplex {
RawSimplexCollection::RawSimplexCollection(SimplexCollection&& sc)
{
    add(sc);
}

std::vector<RawSimplex> RawSimplexCollection::simplex_vector(const int64_t dimension) const
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

void RawSimplexCollection::add(const Mesh& mesh, const Simplex& simplex)
{
    add(RawSimplex(mesh, simplex));
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
        m_simplices.emplace_back(RawSimplex(mesh, Simplex(mesh, ptype, t)));
    }
}

void RawSimplexCollection::sort_and_clean()
{
    std::sort(m_simplices.begin(), m_simplices.end());
    const auto last = std::unique(m_simplices.begin(), m_simplices.end());
    m_simplices.erase(last, m_simplices.end());
}

bool RawSimplexCollection::contains(const RawSimplex& simplex) const
{
    assert(std::is_sorted(begin(), end()));
    return std::binary_search(begin(), end(), simplex);
}

RawSimplexCollection RawSimplexCollection::get_union(
    const RawSimplexCollection& collection_a,
    const RawSimplexCollection& collection_b)
{
    assert(std::is_sorted(collection_a.begin(), collection_a.end()));
    assert(std::is_sorted(collection_b.begin(), collection_b.end()));
    RawSimplexCollection sc;

    const auto& a = collection_a.m_simplices;
    const auto& b = collection_b.m_simplices;

    std::set_union(a.cbegin(), a.cend(), b.cbegin(), b.cend(), std::back_inserter(sc.m_simplices));

    return sc;
}

RawSimplexCollection RawSimplexCollection::get_intersection(
    const RawSimplexCollection& collection_a,
    const RawSimplexCollection& collection_b)
{
    RawSimplexCollection sc;

    const auto& a = collection_a;
    const auto& b = collection_b;

    std::set_intersection(
        a.cbegin(),
        a.cend(),
        b.cbegin(),
        b.cend(),
        std::back_inserter(sc.m_simplices));

    return sc;
}

bool RawSimplexCollection::are_simplex_collections_equal(
    const RawSimplexCollection& collection_a,
    const RawSimplexCollection& collection_b)
{
    if (collection_a.m_simplices.size() != collection_b.m_simplices.size()) {
        return false;
    }
    RawSimplexCollection sc_union = RawSimplexCollection::get_union(collection_a, collection_b);
    return sc_union.m_simplices.size() == collection_a.m_simplices.size();
}

} // namespace wmtk::simplex
