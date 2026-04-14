#include "SimplexCollection.hpp"

#include <algorithm>
#include <cassert>

namespace wmtk::simplex {
const std::vector<Vertex>& SimplexCollection::vertices() const
{
    return m_v;
}

const std::vector<Edge>& SimplexCollection::edges() const
{
    return m_e;
}

const std::vector<Face>& SimplexCollection::faces() const
{
    return m_f;
}

const std::vector<Tet>& SimplexCollection::tets() const
{
    return m_t;
}

void SimplexCollection::add(const SimplexCollection& simplex_collection)
{
    m_v.insert(m_v.end(), simplex_collection.m_v.begin(), simplex_collection.m_v.end());
    m_e.insert(m_e.end(), simplex_collection.m_e.begin(), simplex_collection.m_e.end());
    m_f.insert(m_f.end(), simplex_collection.m_f.begin(), simplex_collection.m_f.end());
    m_t.insert(m_t.end(), simplex_collection.m_t.begin(), simplex_collection.m_t.end());
}

void SimplexCollection::sort_and_clean()
{
    auto f = [](auto& vec) {
        std::sort(vec.begin(), vec.end());
        const auto last = std::unique(vec.begin(), vec.end());
        vec.erase(last, vec.end());
    };
    f(m_v);
    f(m_e);
    f(m_f);
    f(m_t);
}

SimplexCollection SimplexCollection::get_union(
    const SimplexCollection& collection_a,
    const SimplexCollection& collection_b)
{
    auto f = [](auto& a, auto& b) {
        assert(std::is_sorted(a.begin(), a.end()));
        assert(std::is_sorted(b.begin(), b.end()));

        std::decay_t<decltype(a)> c;
        std::set_union(a.cbegin(), a.cend(), b.cbegin(), b.cend(), std::back_inserter(c));
        return c;
    };

    SimplexCollection sc;
    sc.m_v = f(collection_a.m_v, collection_b.m_v);
    sc.m_e = f(collection_a.m_e, collection_b.m_e);
    sc.m_f = f(collection_a.m_f, collection_b.m_f);
    sc.m_t = f(collection_a.m_t, collection_b.m_t);

    return sc;
}

SimplexCollection SimplexCollection::get_intersection(
    const SimplexCollection& collection_a,
    const SimplexCollection& collection_b)
{
    auto f = [](auto& a, auto& b) {
        assert(std::is_sorted(a.begin(), a.end()));
        assert(std::is_sorted(b.begin(), b.end()));

        std::decay_t<decltype(a)> c;
        std::set_intersection(a.cbegin(), a.cend(), b.cbegin(), b.cend(), std::back_inserter(c));
        return c;
    };

    SimplexCollection sc;
    sc.m_v = f(collection_a.m_v, collection_b.m_v);
    sc.m_e = f(collection_a.m_e, collection_b.m_e);
    sc.m_f = f(collection_a.m_f, collection_b.m_f);
    sc.m_t = f(collection_a.m_t, collection_b.m_t);

    return sc;
}

bool SimplexCollection::are_simplex_collections_equal(
    const SimplexCollection& collection_a,
    const SimplexCollection& collection_b)
{
    if (collection_a.size() != collection_b.size()) {
        return false;
    }
    if (collection_a.m_v != collection_b.m_v) {
        return false;
    }
    if (collection_a.m_e != collection_b.m_e) {
        return false;
    }
    if (collection_a.m_f != collection_b.m_f) {
        return false;
    }
    return collection_a.m_t == collection_b.m_t;
}

std::vector<Face> SimplexCollection::faces_with_edge(const Edge& e) const
{
    std::vector<Face> fs;
    for (const Face& f : m_f) {
        if (f.contains(e)) {
            fs.push_back(f);
        }
    }
    return fs;
}

size_t SimplexCollection::size() const
{
    return m_v.size() + m_e.size() + m_f.size() + m_t.size();
}

bool SimplexCollection::operator==(const SimplexCollection& that) const
{
    return (m_v == that.m_v) && (m_e == that.m_e) && (m_f == that.m_f) && (m_t == that.m_t);
}
bool SimplexCollection::operator!=(const SimplexCollection& that) const
{
    return !(*this == that);
}


} // namespace wmtk::simplex
