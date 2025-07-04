#pragma once

#include <algorithm>
#include <array>
#include <cstdarg>
#include <initializer_list>
#include <iterator>
#include <vector>

namespace wmtk::simplex {

/**
 * A simplex represented by its vertex IDs.
 */
template <int N>
class RawSimplex
{
public:
    RawSimplex() = default;
    RawSimplex(std::array<size_t, N>&& vertices)
        : m_vertices{std::move(vertices)}
    {
        std::sort(m_vertices.begin(), m_vertices.end());
    }

    RawSimplex& operator=(const RawSimplex& o) = default;
    bool operator==(const RawSimplex& o) const { return m_vertices == o.m_vertices; }
    bool operator<(const RawSimplex& o) const { return m_vertices < o.m_vertices; }

    const std::array<size_t, N>& vertices() const { return m_vertices; }

protected:
    std::array<size_t, N> m_vertices;
};

class Vertex : public RawSimplex<1>
{
public:
    Vertex() = default;
    Vertex(size_t v0);
    Vertex(const Vertex& o) = default;
    size_t id() const { return vertices()[0]; }
};

class Edge : public RawSimplex<2>
{
public:
    Edge() = default;
    Edge(size_t v0, size_t v1);
    Edge(const Edge&) = default;

    Vertex opposite_vertex(const size_t excluded_id) const;
    Vertex opposite_vertex(const Vertex& v) const;
};

class Face : public RawSimplex<3>
{
public:
    Face() = default;
    Face(size_t v0, size_t v1, size_t v2);
    Face(const Face&) = default;

    Edge opposite_edge(const int64_t excluded_id) const;
    Edge opposite_edge(const Vertex& v) const;
    Vertex opposite_vertex(const Edge& e) const;
};

class Tet : public RawSimplex<4>
{
public:
    Tet() = default;
    Tet(size_t v0, size_t v1, size_t v2, size_t v3);
    Tet(const Tet&) = default;

    Face opposite_face(const int64_t excluded_id) const;
    Face opposite_face(const Vertex& v) const;
    Edge opposite_edge(const Edge& e) const;
    Vertex opposite_vertex(const Face& f) const;
};

} // namespace wmtk::simplex
