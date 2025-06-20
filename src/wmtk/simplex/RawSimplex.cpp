#include "RawSimplex.hpp"

#include <algorithm>

#include "RawSimplexCollection.hpp"

namespace wmtk::simplex {


// template <int N, int M>
// bool RawSimplex<N>::operator==(const RawSimplex<M>& o) const
//{
//    // static_assert(N == M);
//    if constexpr (N != M) {
//        return false;
//    }
//
//    return std::equal(
//        m_vertices.begin(),
//        m_vertices.end(),
//        o.m_vertices.begin(),
//        o.m_vertices.end());
//}

// template <int N, int M>
// bool RawSimplex<N>::operator<(const RawSimplex<M>& o) const
//{
//     if constexpr (N != M) {
//         return N < M;
//     }
//
//     return std::lexicographical_compare(
//         m_vertices.begin(),
//         m_vertices.end(),
//         o.m_vertices.begin(),
//         o.m_vertices.end());
// }

// template <int N>
// template <int N, int M>
// RawSimplex<N - M> RawSimplex<N>::opposite_face(const RawSimplex<M>& face)
//{
//     const auto& s_v = m_vertices;
//     const auto& f_v = face.m_vertices;
//
//     assert(f_v.size() <= s_v.size());
//
//     std::vector<int64_t> o_v;
//     o_v.reserve(s_v.size() - f_v.size());
//
//     std::set_difference(
//         s_v.begin(),
//         s_v.end(),
//         f_v.begin(),
//         f_v.end(),
//         std::inserter(o_v, o_v.begin()));
//
//     assert(o_v.size() == s_v.size() - f_v.size());
//
//     return RawSimplex(std::move(o_v));
// }

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

// template class RawSimplex<1>;
// template class RawSimplex<2>;
// template class RawSimplex<3>;
// template class RawSimplex<4>;

} // namespace wmtk::simplex
