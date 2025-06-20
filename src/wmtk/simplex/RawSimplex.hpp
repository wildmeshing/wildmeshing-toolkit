#pragma once

#include <algorithm>
#include <array>
#include <cstdarg>
#include <initializer_list>
#include <iterator>
#include <vector>

namespace wmtk::simplex {

class RawSimplexCollection;

/**
 * A meshless implementation of the simplex that just stores an array of ids.
 * It is used for scenarios where a mesh does not exist.
 *
 * It must not be used for "degenerated" simplices that contain the same vertex several times, e.g.
 * {0,1,2,0} because it would be reduced to {0,1,2}.
 */
template <int N>
class RawSimplex
{
    friend class RawSimplexCollection;

public:
    const int DIM = N - 1; // dimension of the simplex

    RawSimplex() = default;

    RawSimplex(std::array<size_t, N>&& vertices);
    RawSimplex(size_t v0);
    RawSimplex(size_t v0, size_t v1);
    RawSimplex(size_t v0, size_t v1, size_t v2);
    RawSimplex(size_t v0, size_t v1, size_t v2, size_t v3);

    RawSimplex& operator=(const RawSimplex& o);

    template <int M>
    bool operator==(const RawSimplex<M>& o) const;

    template <int M>
    bool operator<(const RawSimplex<M>& o) const;

    /**
     * @brief Get the face opposite to the given vertex.
     * The face consists of all vertices except for the given one.
     *
     * @param excluded_id The vertex index that is not included in the returned simplex.
     *
     * @return RawSimplex representing the face opposing the given vertex.
     */
    RawSimplex<N - 1> opposite_face(const int64_t excluded_id);

    /**
     * @brief Get the face opposite to the given face.
     * The opposite face consists of all vertices except for the one that belong to the given face.
     *
     * @param face A RawSimplex representing that part of the simplex that is not included in the
     * returned simplex.
     *
     * @return RawSimplex representing the face opposing the given face.
     */
    template <int M>
    RawSimplex<N - M> opposite_face(const RawSimplex<M>& face);

    const std::array<size_t, N>& vertices() const { return m_vertices; }

private:
    std::array<size_t, N> m_vertices;
};

template <int N>
inline RawSimplex<N>::RawSimplex(std::array<size_t, N>&& vertices)
    : m_vertices{std::move(vertices)}
{
    std::sort(m_vertices.begin(), m_vertices.end());
}

template <int N>
inline RawSimplex<N>::RawSimplex(size_t v0)
{
    static_assert(N == 1, "RawSimplex must be initialized with 1 vertex ids");
    m_vertices[0] = v0;
}

template <int N>
inline RawSimplex<N>::RawSimplex(size_t v0, size_t v1)
{
    static_assert(N == 2, "RawSimplex must be initialized with 2 vertex ids");
    if (v0 < v1) {
        m_vertices[0] = v0;
        m_vertices[1] = v1;
    } else {
        m_vertices[0] = v1;
        m_vertices[1] = v0;
    }
}

template <int N>
inline RawSimplex<N>::RawSimplex(size_t v0, size_t v1, size_t v2)
{
    static_assert(N == 3, "RawSimplex must be initialized with 3 vertex ids");
    m_vertices[0] = v0;
    m_vertices[1] = v1;
    m_vertices[2] = v2;
    std::sort(m_vertices.begin(), m_vertices.end());
}

template <int N>
inline RawSimplex<N>::RawSimplex(size_t v0, size_t v1, size_t v2, size_t v3)
{
    static_assert(N == 4, "RawSimplex must be initialized with 4 vertex ids");
    m_vertices[0] = v0;
    m_vertices[1] = v1;
    m_vertices[2] = v2;
    m_vertices[3] = v3;
    std::sort(m_vertices.begin(), m_vertices.end());
}

template <int N>
inline RawSimplex<N>& RawSimplex<N>::operator=(const RawSimplex<N>& o)
{
    m_vertices = o.m_vertices;
    return *this;
}

template <int N>
template <int M>
bool RawSimplex<N>::operator==(const RawSimplex<M>& o) const
{
    if constexpr (N != M) {
        return false;
    } else {
        return std::equal(
            m_vertices.begin(),
            m_vertices.end(),
            o.m_vertices.begin(),
            o.m_vertices.end());
    }
}

template <int N>
template <int M>
bool RawSimplex<N>::operator<(const RawSimplex<M>& o) const
{
    if constexpr (N != M) {
        return N < M;
    } else {
        return std::lexicographical_compare(
            m_vertices.begin(),
            m_vertices.end(),
            o.m_vertices.begin(),
            o.m_vertices.end());
    }
}

template <int N>
inline RawSimplex<N - 1> RawSimplex<N>::opposite_face(const int64_t excluded_id)
{
    static_assert(N > 1);

    std::array<size_t, N - 1> face_ids;

    size_t counter = 0;
    for (size_t i = 0; i < N; ++i) {
        if (m_vertices[i] != excluded_id) {
            face_ids[counter++] = m_vertices[i];
        }
    }
    assert(counter == N - 1);

    RawSimplex<N - 1> face(std::move(face_ids));

    return face;
}

template <int N>
template <int M>
RawSimplex<N - M> RawSimplex<N>::opposite_face(const RawSimplex<M>& face)
{
    const auto& s_v = m_vertices;
    const auto& f_v = face.vertices();

    assert(f_v.size() <= s_v.size());

    std::array<size_t, N - M> o_v;

    std::set_difference(s_v.begin(), s_v.end(), f_v.begin(), f_v.end(), o_v.begin());

    assert(o_v.size() == s_v.size() - f_v.size());

    return RawSimplex<N - M>(std::move(o_v));
}

// RawSimplexCollection RawSimplex<N>::faces()
//{
//     const auto& v = m_vertices;
//
//     std::vector<RawSimplex> faces;
//
//     switch (dimension()) {
//    case 0: { // simplex is a vertex
//        break;
//    }
//    case 1: { // simplex is an edge
//        faces.reserve(2);
//        faces.emplace_back(RawSimplex({v[0]}));
//        faces.emplace_back(RawSimplex({v[1]}));
//        break;
//    }
//    case 2: { // simplex is a triangle
//        faces.reserve(6);
//        faces.emplace_back(RawSimplex({v[0]}));
//        faces.emplace_back(RawSimplex({v[1]}));
//        faces.emplace_back(RawSimplex({v[2]}));
//        faces.emplace_back(RawSimplex({v[0], v[1]}));
//        faces.emplace_back(RawSimplex({v[0], v[2]}));
//        faces.emplace_back(RawSimplex({v[1], v[2]}));
//        break;
//    }
//    case 3: { // simplex is a tetrahedron
//        faces.reserve(14);
//        faces.emplace_back(RawSimplex({v[0]}));
//        faces.emplace_back(RawSimplex({v[1]}));
//        faces.emplace_back(RawSimplex({v[2]}));
//        faces.emplace_back(RawSimplex({v[3]}));
//        faces.emplace_back(RawSimplex({v[0], v[1]}));
//        faces.emplace_back(RawSimplex({v[0], v[2]}));
//        faces.emplace_back(RawSimplex({v[0], v[3]}));
//        faces.emplace_back(RawSimplex({v[1], v[2]}));
//        faces.emplace_back(RawSimplex({v[1], v[3]}));
//        faces.emplace_back(RawSimplex({v[2], v[3]}));
//        faces.emplace_back(RawSimplex({v[0], v[1], v[2]}));
//        faces.emplace_back(RawSimplex({v[0], v[1], v[3]}));
//        faces.emplace_back(RawSimplex({v[0], v[2], v[3]}));
//        faces.emplace_back(RawSimplex({v[1], v[2], v[3]}));
//        break;
//    }
//    default: assert(false); // "Unexpected dimension in RawSimplex."
//    }
//
//    return RawSimplexCollection(std::move(faces));
//}

} // namespace wmtk::simplex
