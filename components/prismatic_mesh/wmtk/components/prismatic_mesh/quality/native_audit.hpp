#pragma once
#include "native_mixed.hpp"
namespace wmtk::components::prismatic_mesh::quality::native {
inline std::vector<size_t> cycle(std::vector<size_t> v)
{
    std::rotate(v.begin(), std::min_element(v.begin(), v.end()), v.end());
    return v;
}
using Key = std::array<size_t, 5>;
inline Key facekey(const std::vector<size_t>& v)
{
    Key k{};
    k[0] = v.size();
    std::copy(v.begin(), v.end(), k.begin() + 1);
    std::sort(k.begin() + 1, k.begin() + 1 + v.size());
    return k;
}
inline double volume(const Mesh& m)
{
    long double sum = 0;
    for (const auto& c : m.cells) {
        if (c.type == 4)
            sum += c.minimum / 6;
        else if (c.type == 6)
            for (const auto& q : quadrature(6, 2))
                sum += q.w * matrix(m.p, c, q).determinant() * std::sqrt(3.) / 4;
        else {
            const auto& p = m.p;
            const auto& v = c.v;
            M a;
            const V p1 = p[v[1]] - p[v[0]], p2 = p[v[2]] - p[v[0]], p3 = p[v[3]] - p[v[0]];
            a.col(0) = (p1 + p2 - p3) / 2;
            a.col(1) = (-p1 + p2 + p3) / 2;
            a.col(2) = std::sqrt(2.) * (p[v[4]] - p[v[0]] - (p1 + p2 + p3) / 4);
            sum += a.determinant() / (3 * std::sqrt(2.));
        }
    }
    return double(sum);
}
} // namespace wmtk::components::prismatic_mesh::quality::native
