#pragma once
#include "common.hpp"
namespace wmtk::components::prismatic_mesh::quality {
using Tri = std::array<size_t, 3>;
inline Tri oriented_tri(const std::vector<size_t>& v)
{
    auto a = native::cycle(v);
    return {a[0], a[1], a[2]};
}
inline double triangle_shape(const Mesh& m, const Tri& f)
{
    const V a = m.p[f[1]] - m.p[f[0]], b = m.p[f[2]] - m.p[f[0]];
    const double s = a.squaredNorm() + b.squaredNorm() + (a - b).squaredNorm();
    return s > 0 ? 2 * std::sqrt(3.) * a.cross(b).norm() / s : 0.;
}
// Boundary of the actual shell-tet complex, including tet/native interfaces.
// Vertex IDs remain stable when replace_patch reorders volume cells.
struct BoundarySurface
{
    std::map<Key, std::set<Tri>> contributions;
    std::map<Key, Tri> triangles;
    std::map<Edge, std::set<Key>> edges;
    std::vector<std::set<size_t>> neighbors;
    explicit BoundarySurface(const Mesh& m)
        : neighbors(m.p.size())
    {
        for (const auto& c : m.cells) change(c, true);
    }
    void expose(const Key& k, const Tri& f, bool add)
    {
        if (add)
            triangles[k] = f;
        else
            triangles.erase(k);
        for (int j = 0; j < 3; ++j) {
            const auto e = native::edge(f[j], f[(j + 1) % 3]);
            if (add) {
                auto& fs = edges[e];
                if (fs.empty()) {
                    neighbors[e[0]].insert(e[1]);
                    neighbors[e[1]].insert(e[0]);
                }
                fs.insert(k);
            } else {
                auto it = edges.find(e);
                if (it == edges.end() || !it->second.erase(k))
                    throw std::runtime_error("boundary_index_edge_missing");
                if (it->second.empty()) {
                    edges.erase(it);
                    neighbors[e[0]].erase(e[1]);
                    neighbors[e[1]].erase(e[0]);
                }
            }
        }
    }
    void change(const Cell& c, bool add)
    {
        if (c.type != 4 || c.region != 2) return;
        for (const auto& local : native::faces.at(4)) {
            std::vector<size_t> v;
            for (int j : local) v.push_back(c.v[j]);
            const auto f = oriented_tri(v);
            const auto k = native::facekey(v);
            auto& all = contributions[k];
            if (all.size() == 1) expose(k, *all.begin(), false);
            if (add) {
                if (!all.insert(f).second) throw std::runtime_error("duplicate_shell_tet_face");
            } else if (!all.erase(f))
                throw std::runtime_error("boundary_index_face_missing");
            if (all.size() > 2) throw std::runtime_error("nonmanifold_shell_tet_face");
            if (all.size() == 1) expose(k, *all.begin(), true);
            if (all.empty()) contributions.erase(k);
        }
    }
    void replace(const std::vector<Cell>& old, const std::vector<Cell>& next)
    {
        for (const auto& c : old) change(c, false);
        for (const auto& c : next) change(c, true);
    }
    Json stats(const Mesh& m) const
    {
        std::vector<double> q;
        double sum = 0;
        size_t low = 0;
        for (const auto& [k, f] : triangles) {
            q.push_back(triangle_shape(m, f));
            sum += q.back();
            low += q.back() < .4;
        }
        std::sort(q.begin(), q.end());
        Json hist = Json::object();
        size_t verts = 0, regular = 0;
        for (const auto& ns : neighbors)
            if (!ns.empty()) {
                ++verts;
                regular += ns.size() == 6;
                const auto key = std::to_string(ns.size());
                hist[key] = hist.value(key, size_t(0)) + 1;
            }
        return {
            {"triangles", q.size()},
            {"vertices", verts},
            {"minimum_shape", q.empty() ? 0 : q[0]},
            {"p05_shape", q.empty() ? 0 : q[size_t(.05 * (q.size() - 1))]},
            {"mean_shape", q.empty() ? 0 : sum / q.size()},
            {"shape_below_0_4", low},
            {"degree6_vertices", regular},
            {"degree_histogram", hist}};
    }
};
struct SurfaceDecision
{
    double before_min = 1, after_min = 1, before_mean = 0, after_mean = 0, before_degree = 0,
           after_degree = 0;
    bool accepted = false;
    Json json() const
    {
        return {
            {"minimum_triangle_shape_before", before_min},
            {"minimum_triangle_shape_after", after_min},
            {"mean_triangle_shape_before", before_mean},
            {"mean_triangle_shape_after", after_mean},
            {"degree_energy_before", before_degree},
            {"degree_energy_after", after_degree},
            {"criterion", "triangle shape first; degree toward 6 breaks quality ties"}};
    }
};
inline SurfaceDecision surface_decision(
    const Mesh& m,
    const BoundarySurface* surface,
    Edge edge,
    Edge newedge,
    const std::vector<Tri>& old,
    const std::vector<Tri>& next)
{
    SurfaceDecision d;
    for (const auto& f : old) {
        const auto q = triangle_shape(m, f);
        d.before_min = std::min(d.before_min, q);
        d.before_mean += q / old.size();
    }
    for (const auto& f : next) {
        const auto q = triangle_shape(m, f);
        d.after_min = std::min(d.after_min, q);
        d.after_mean += q / next.size();
    }
    if (surface) {
        for (size_t v : edge) {
            const double x = double(surface->neighbors.at(v).size()) - 6;
            d.before_degree += x * x;
            d.after_degree += (x - 1) * (x - 1);
        }
        for (size_t v : newedge) {
            const double x = double(surface->neighbors.at(v).size()) - 6;
            d.before_degree += x * x;
            d.after_degree += (x + 1) * (x + 1);
        }
    }
    const double eps = 1e-10;
    d.accepted = d.after_min > d.before_min + eps ||
                 (d.after_min >= d.before_min - eps &&
                  (d.after_mean > d.before_mean + eps || (d.after_mean >= d.before_mean - eps &&
                                                          d.after_degree < d.before_degree - eps)));
    return d;
}
} // namespace wmtk::components::prismatic_mesh::quality
