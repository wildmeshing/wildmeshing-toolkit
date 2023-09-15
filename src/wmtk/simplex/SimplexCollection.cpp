#include "SimplexCollection.hpp"

#include <algorithm>

namespace wmtk::simplex {


std::vector<Simplex> SimplexCollection::simplex_vector(const PrimitiveType& ptype) const
{
    std::vector<Simplex> simplices;
    simplices.reserve(m_simplices.size() / 3); // giving the vector some (hopefully) resonable size

    // add simplices to the vector
    for (const Simplex& s : m_simplices) {
        if (s.primitive_type() == ptype) {
            simplices.emplace_back(s);
        }
    }

    return simplices;
}

void SimplexCollection::add(const Simplex& simplex)
{
    m_simplices.push_back(simplex);
}

void SimplexCollection::add(const SimplexCollection& simplex_collection)
{
    const auto& s = simplex_collection.m_simplices;
    m_simplices.insert(m_simplices.end(), s.begin(), s.end());
}

void SimplexCollection::sort_and_clean()
{
    std::sort(m_simplices.begin(), m_simplices.end(), m_simplex_is_less);
    const auto last = std::unique(m_simplices.begin(), m_simplices.end(), m_simplex_is_equal);
    m_simplices.erase(last, m_simplices.end());
}

bool SimplexCollection::contains(const Simplex& simplex)
{
    // TODO this is O(n) but can and should be done in O(log n)
    for (const Simplex& s : m_simplices) {
        if (m_mesh.simplex_is_equal(s, simplex)) {
            return true;
        }
    }
    return false;
}

SimplexCollection SimplexCollection::get_union(
    const SimplexCollection& collection_a,
    const SimplexCollection& collection_b)
{
    SimplexCollection sc(collection_a.m_mesh);

    const auto& a = collection_a.m_simplices;
    const auto& b = collection_b.m_simplices;

    std::set_union(
        a.cbegin(),
        a.cend(),
        b.cbegin(),
        b.cend(),
        std::back_inserter(sc.m_simplices),
        sc.m_simplex_is_less);

    return sc;
}

SimplexCollection SimplexCollection::get_intersection(
    const SimplexCollection& collection_a,
    const SimplexCollection& collection_b)
{
    SimplexCollection sc(collection_a.m_mesh);

    const auto& a = collection_a.m_simplices;
    const auto& b = collection_b.m_simplices;

    std::set_intersection(
        a.cbegin(),
        a.cend(),
        b.cbegin(),
        b.cend(),
        std::back_inserter(sc.m_simplices),
        sc.m_simplex_is_less);

    return sc;
}

} // namespace wmtk::simplex