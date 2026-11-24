#pragma once
#include "common.hpp"
namespace wmtk::components::prismatic_mesh::quality {
struct MixedReference
{
    double h = 0, base_length = 0;
    std::vector<V> normal;
    std::vector<double> height;
    MixedReference() = default;
    MixedReference(
        const Mesh& m,
        double thickness,
        double length,
        const std::vector<V>& supplied_normals)
        : h(thickness)
        , base_length(length)
        , normal(m.p.size(), V::Zero())
        , height(m.p.size(), 0)
    {
        if (!supplied_normals.empty()) {
            if (supplied_normals.size() != m.p.size())
                throw std::runtime_error("invalid_offset_normals_size");
            normal = supplied_normals;
            for (auto& n : normal) {
                if (!n.allFinite()) throw std::invalid_argument("nonfinite_reference_normal");
                if (n.norm() > 1e-10) n.normalize();
            }
        }
        extend(m);
    }
    void extend(const Mesh& m)
    {
        normal.resize(m.p.size(), V::Zero());
        height.resize(m.p.size(), 0);
        for (size_t v = 0; v < m.p.size(); ++v)
            if (std::isfinite(m.thickness[v]) && m.thickness[v] > 0) {
                if (normal[v].norm() < .5) {
                    V n = m.initial[v] - m.base[v];
                    if (n.norm() > 1e-12) normal[v] = n.normalized();
                }
                if (height[v] == 0) height[v] = h;
            }
    }
    bool offset(const Mesh& m, size_t v) const
    {
        return !m.input[v] && !m.outer[v] && std::isfinite(m.thickness[v]) && m.thickness[v] > 0 &&
               normal[v].norm() > .5 && height[v] > 0;
    }
    void midpoint(const Mesh& m, size_t v, Edge parents)
    {
        normal.resize(m.p.size(), V::Zero());
        height.resize(m.p.size(), 0);
        const double a = m.input[parents[0]] ? 0 : height[parents[0]],
                     b = m.input[parents[1]] ? 0 : height[parents[1]];
        height[v] = .5 * (a + b);
        V n = a * normal[parents[0]] + b * normal[parents[1]];
        if (n.norm() > 1e-12) normal[v] = n.normalized();
    }
    V anchor(const Mesh& m, size_t v) const
    {
        return m.correspondence[v] >= 0 ? m.p.at(m.correspondence[v]) : m.base[v];
    }
    double height_error(const Mesh& m, size_t v) const
    {
        return normal[v].dot(m.p[v] - anchor(m, v)) / height[v] - 1;
    }
    V project(const Mesh& m, size_t v, const V& raw) const
    {
        if (!offset(m, v)) return raw;
        const V n = normal[v], a = anchor(m, v);
        const double band = std::max(.25, std::abs(height_error(m, v))),
                     lo = height[v] * (1 - band), hi = height[v] * (1 + band), z = n.dot(raw - a);
        return raw + (std::clamp(z, lo, hi) - z) * n;
    }
};
} // namespace wmtk::components::prismatic_mesh::quality
