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

const Mesh& SimplexCollection::mesh() const
{
    return m_mesh;
}

std::vector<Tuple> SimplexCollection::simplex_vector_tuples(PrimitiveType ptype) const
{
    std::vector<Tuple> tuples;
    tuples.reserve(m_simplices.size() / 3); // giving the vector some (hopefully) resonable size

    // add simplices to the vector
    for (const Simplex& s : m_simplices) {
        if (s.primitive_type() == ptype) {
            tuples.emplace_back(s.tuple());
        }
    }

    return tuples;
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

void SimplexCollection::add(const PrimitiveType& ptype, const std::vector<Tuple>& tuple_vec)
{
    m_simplices.reserve(m_simplices.size() + tuple_vec.size());

    for (const Tuple& t : tuple_vec) {
        m_simplices.emplace_back(Simplex(ptype, t));
    }
}

void SimplexCollection::sort_and_clean()
{
    std::sort(m_simplices.begin(), m_simplices.end(), m_simplex_is_less);
    const auto last = std::unique(m_simplices.begin(), m_simplices.end(), m_simplex_is_equal);
    m_simplices.erase(last, m_simplices.end());
}

void SimplexCollection::sort()
{
    std::sort(m_simplices.begin(), m_simplices.end(), m_simplex_is_less);
}


bool SimplexCollection::contains(const Simplex& simplex) const
{
    assert(std::is_sorted(m_simplices.begin(), m_simplices.end(), m_simplex_is_less));
    return std::binary_search(m_simplices.begin(), m_simplices.end(), simplex, m_simplex_is_less);
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

bool SimplexCollection::are_simplex_collections_equal(
    const SimplexCollection& collection_a,
    const SimplexCollection& collection_b)
{
    if (collection_a.m_simplices.size() != collection_b.m_simplices.size()) {
        return false;
    }
    SimplexCollection sc_union = SimplexCollection::get_union(collection_a, collection_b);
    return sc_union.m_simplices.size() == collection_a.m_simplices.size();
}

bool SimplexCollection::operator==(const SimplexCollection& other) const
{
    return are_simplex_collections_equal(*this, other);
}

} // namespace wmtk::simplex
