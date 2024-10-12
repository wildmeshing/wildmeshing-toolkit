#include "IdSimplexCollection.hpp"

#include <algorithm>

namespace wmtk::simplex {


std::vector<IdSimplex> IdSimplexCollection::simplex_vector(const PrimitiveType& ptype) const
{
    std::vector<IdSimplex> simplices;
    simplices.reserve(m_simplices.size() / 3); // giving the vector some (hopefully) resonable size

    // add simplices to the vector
    for (const IdSimplex& s : m_simplices) {
        if (s.primitive_type() == ptype) {
            simplices.emplace_back(s);
        }
    }

    return simplices;
}

const Mesh& IdSimplexCollection::mesh() const
{
    return m_mesh;
}

std::vector<Tuple> IdSimplexCollection::simplex_vector_tuples(PrimitiveType ptype) const
{
    std::vector<Tuple> tuples;
    tuples.reserve(m_simplices.size() / 3); // giving the vector some (hopefully) resonable size

    // add simplices to the vector
    for (const IdSimplex& s : m_simplices) {
        if (s.primitive_type() == ptype) {
            tuples.emplace_back(m_mesh.get_tuple_from_id_simplex(s));
        }
    }

    return tuples;
}

void IdSimplexCollection::add(const IdSimplex& simplex)
{
    m_simplices.push_back(simplex);
}

void IdSimplexCollection::add(const IdSimplexCollection& simplex_collection)
{
    const auto& s = simplex_collection.m_simplices;
    m_simplices.insert(m_simplices.end(), s.begin(), s.end());
}

void IdSimplexCollection::add(const PrimitiveType ptype, const std::vector<Tuple>& tuple_vec)
{
    m_simplices.reserve(m_simplices.size() + tuple_vec.size());

    for (const Tuple& t : tuple_vec) {
        m_simplices.emplace_back(Simplex(mesh(), ptype, t));
    }
}

void IdSimplexCollection::add(const PrimitiveType ptype, const Tuple& tuple)
{
    m_simplices.emplace_back(Simplex(mesh(), ptype, tuple));
}

void IdSimplexCollection::sort_and_clean()
{
    std::sort(m_simplices.begin(), m_simplices.end());
    const auto last = std::unique(m_simplices.begin(), m_simplices.end());
    m_simplices.erase(last, m_simplices.end());
}

void IdSimplexCollection::sort()
{
    std::sort(m_simplices.begin(), m_simplices.end());
}


bool IdSimplexCollection::contains(const IdSimplex& simplex) const
{
    assert(std::is_sorted(m_simplices.begin(), m_simplices.end()));
    return std::binary_search(m_simplices.begin(), m_simplices.end(), simplex);
}

IdSimplexCollection IdSimplexCollection::get_union(
    const IdSimplexCollection& collection_a,
    const IdSimplexCollection& collection_b)
{
    IdSimplexCollection sc(collection_a.m_mesh);

    const auto& a = collection_a.m_simplices;
    const auto& b = collection_b.m_simplices;

    std::set_union(a.cbegin(), a.cend(), b.cbegin(), b.cend(), std::back_inserter(sc.m_simplices));

    return sc;
}

IdSimplexCollection IdSimplexCollection::get_intersection(
    const IdSimplexCollection& collection_a,
    const IdSimplexCollection& collection_b)
{
    IdSimplexCollection sc(collection_a.m_mesh);

    const auto& a = collection_a.m_simplices;
    const auto& b = collection_b.m_simplices;

    std::set_intersection(
        a.cbegin(),
        a.cend(),
        b.cbegin(),
        b.cend(),
        std::back_inserter(sc.m_simplices));

    return sc;
}

bool IdSimplexCollection::are_simplex_collections_equal(
    const IdSimplexCollection& collection_a,
    const IdSimplexCollection& collection_b)
{
    if (collection_a.m_simplices.size() != collection_b.m_simplices.size()) {
        return false;
    }
    IdSimplexCollection sc_union = IdSimplexCollection::get_union(collection_a, collection_b);
    return sc_union.m_simplices.size() == collection_a.m_simplices.size();
}

bool IdSimplexCollection::operator==(const IdSimplexCollection& other) const
{
    return are_simplex_collections_equal(*this, other);
}

void IdSimplexCollection::reserve(const size_t new_cap)
{
    m_simplices.reserve(new_cap);
}

} // namespace wmtk::simplex
