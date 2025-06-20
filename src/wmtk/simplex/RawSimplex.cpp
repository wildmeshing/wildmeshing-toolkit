#include "RawSimplex.hpp"

#include <algorithm>

#include "RawSimplexCollection.hpp"

namespace {
template <int N>
std::array<size_t, N - 1> array_without(const std::array<size_t, N>& a, const int64_t excluded)
{
    static_assert(N > 1);

    std::array<size_t, N - 1> new_a;

    size_t counter = 0;
    for (size_t i = 0; i < N; ++i) {
        if (a[i] != excluded) {
            new_a[counter++] = a[i];
        }
    }
    assert(counter == N - 1);

    return new_a;
}

template <int N, int M>
std::array<size_t, N - M> array_without(
    const std::array<size_t, N>& a,
    const std::array<size_t, M>& excluded)
{
    const auto& s_v = a;
    const auto& f_v = excluded;

    assert(f_v.size() <= s_v.size());

    std::array<size_t, N - M> o_v;

    std::set_difference(s_v.begin(), s_v.end(), f_v.begin(), f_v.end(), o_v.begin());

    assert(o_v.size() == s_v.size() - f_v.size());

    return o_v;
}

} // namespace

namespace wmtk::simplex {

Vertex::Vertex(size_t v0)
{
    m_vertices[0] = v0;
}

Edge::Edge(size_t v0, size_t v1)
{
    if (v0 < v1) {
        m_vertices[0] = v0;
        m_vertices[1] = v1;
    } else {
        m_vertices[0] = v1;
        m_vertices[1] = v0;
    }
}

Vertex Edge::opposite_vertex(const int64_t excluded_id)
{
    auto a = array_without(m_vertices, excluded_id);
    return Vertex(a[0]);
}

Vertex Edge::opposite_vertex(const Vertex& v)
{
    auto a = array_without(m_vertices, v.vertices()[0]);
    return Vertex(a[0]);
}

Face::Face(size_t v0, size_t v1, size_t v2)
{
    m_vertices[0] = v0;
    m_vertices[1] = v1;
    m_vertices[2] = v2;
    std::sort(m_vertices.begin(), m_vertices.end());
}

Edge Face::opposite_edge(const int64_t excluded_id)
{
    auto a = array_without(m_vertices, excluded_id);
    return Edge(a[0], a[1]);
}

Edge Face::opposite_edge(const Vertex& v)
{
    auto a = array_without(m_vertices, v.vertices());
    return Edge(a[0], a[1]);
}

Vertex Face::opposite_vertex(const Edge& v)
{
    auto a = array_without(m_vertices, v.vertices()[0]);
    return Vertex(a[0]);
}

Tet::Tet(size_t v0, size_t v1, size_t v2, size_t v3)
{
    m_vertices[0] = v0;
    m_vertices[1] = v1;
    m_vertices[2] = v2;
    m_vertices[3] = v3;
    std::sort(m_vertices.begin(), m_vertices.end());
}

Face Tet::opposite_face(const int64_t excluded_id)
{
    auto a = array_without(m_vertices, excluded_id);
    return Face(a[0], a[1], a[2]);
}

Face Tet::opposite_face(const Vertex& v)
{
    auto a = array_without(m_vertices, v.vertices());
    return Face(a[0], a[1], a[2]);
}

Edge Tet::opposite_edge(const Edge& v)
{
    auto a = array_without(m_vertices, v.vertices());
    return Edge(a[0], a[1]);
}

Vertex Tet::opposite_vertex(const Face& v)
{
    auto a = array_without(m_vertices, v.vertices()[0]);
    return Vertex(a[0]);
}

} // namespace wmtk::simplex
