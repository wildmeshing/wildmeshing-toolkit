#pragma once
#include "common.hpp"
#include "tet_backend.hpp"
namespace wmtk::components::prismatic_mesh::quality {
template <size_t N>
struct GeomHash
{
    size_t operator()(const std::array<double, N>& a) const noexcept
    {
        size_t h = 0;
        for (double d : a) h ^= std::hash<double>{}(d) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};
template <size_t N>
inline std::array<double, 3 * N> positions(
    const std::vector<V>& p,
    const std::array<size_t, N>& ids)
{
    std::array<std::array<double, 3>, N> xyz;
    for (size_t i = 0; i < N; ++i) xyz[i] = {p[ids[i]][0], p[ids[i]][1], p[ids[i]][2]};
    std::sort(xyz.begin(), xyz.end());
    std::array<double, 3 * N> key;
    for (size_t i = 0; i < N; ++i)
        for (int j = 0; j < 3; ++j) key[3 * i + j] = xyz[i][j];
    return key;
}
class Bulk final : public StockFixedBackground
{
    std::unordered_set<std::array<double, 6>, GeomHash<6>> edges;
    std::unordered_set<std::array<double, 9>, GeomHash<9>> faces;
    template <size_t N>
    std::array<double, 3 * N> key(std::array<size_t, N> ids) const
    {
        std::array<std::array<double, 3>, N> xyz;
        for (size_t i = 0; i < N; ++i) {
            const auto& p = m_vertex_attribute[ids[i]].m_posf;
            xyz[i] = {p[0], p[1], p[2]};
        }
        std::sort(xyz.begin(), xyz.end());
        std::array<double, 3 * N> out;
        for (size_t i = 0; i < N; ++i)
            for (int j = 0; j < 3; ++j) out[3 * i + j] = xyz[i][j];
        return out;
    }

public:
    std::function<bool()> cancelled;
    size_t rejected_unrounded_splits = 0;
    Bulk(
        wmtk::OptimizerParameters& params,
        const std::vector<V>& p,
        const std::vector<Tet>& tets,
        const std::vector<uint8_t>& pins,
        const std::vector<double>& sizing,
        const std::vector<std::array<size_t, 3>>& barriers)
        : StockFixedBackground(params, p, tets, pins, "", sizing)
    {
        for (const auto& f : barriers) {
            faces.insert(positions<3>(p, f));
            for (int j = 0; j < 3; ++j) edges.insert(positions<2>(p, {f[j], f[(j + 1) % 3]}));
        }
    }
    bool split_edge_before(const Tuple& t) override
    {
        if (edges.count(key<2>({t.vid(*this), t.switch_vertex(*this).vid(*this)}))) return false;
        return StockFixedBackground::split_edge_before(t);
    }
    void split_after_vertex(size_t v, bool boundary) override
    {
        if (m_vertex_attribute[v].m_is_rounded)
            StockFixedBackground::split_after_vertex(v, boundary);
    }
    bool split_edge_after(const Tuple& t) override
    {
        if (!TetOptimizerMesh::split_edge_after(t)) return false;
        if (!m_vertex_attribute[t.vid(*this)].m_is_rounded) {
            ++rejected_unrounded_splits;
            return false;
        }
        return true;
    }
    bool swap_edge_before(const Tuple& t) override
    {
        if (cancelled && cancelled()) return false;
        if (edges.count(key<2>({t.vid(*this), t.switch_vertex(*this).vid(*this)}))) return false;
        return TetOptimizerMesh::swap_edge_before(t);
    }
    bool swap_face_before(const Tuple& t) override
    {
        if (cancelled && cancelled()) return false;
        if (faces.count(key<3>(get_face_vids(t)))) return false;
        return TetOptimizerMesh::swap_face_before(t);
    }
    bool swap_before_interior(const std::vector<size_t>& tids) override
    {
        if (cancelled && cancelled()) return false;
        std::unordered_set<std::array<double, 9>, GeomHash<9>> seen;
        for (size_t tid : tids)
            for (int j = 0; j < 4; ++j) {
                const auto f = tuple_from_face(tid, j);
                const auto k = key<3>(get_face_vids(f));
                if (faces.count(k) && !seen.insert(k).second) return false;
            }
        return true;
    }
    Json report() const
    {
        auto r = StockFixedBackground::report();
        r["floating_point_safe_splits_only"] = true;
        r["rejected_unrounded_split_proposals"] = rejected_unrounded_splits;
        r["swap_deadline_checked"] = true;
        return r;
    }
};
inline Json remesh_all_tets(
    Mesh& m,
    const Rules& rules,
    double length,
    size_t split_budget,
    Deadline& deadline,
    int passes = 2)
{
    if (std::none_of(m.cells.begin(), m.cells.end(), [](const Cell& c) { return c.type == 4; }))
        return {{"skipped", "no_tetrahedra"}};
    const auto before_faces = faces_of(m.cells);
    const auto original_topology = topology(m, before_faces);
    struct Seed
    {
        std::vector<size_t> oriented;
        int r0 = 0, r1 = 0;
    };
    std::map<Key, Seed> seeds;
    std::vector<std::array<size_t, 3>> barriers;
    std::vector<uint8_t> pins(m.p.size(), false);
    for (size_t v = 0; v < m.p.size(); ++v) pins[v] = m.outer[v] || m.input[v];
    for (const auto& c : m.cells)
        if (c.type != 4)
            for (size_t v : c.v) pins[v] = true;
    for (const auto& [key, f] : before_faces) {
        const bool a = m.cells[f.a].type == 4, b = f.b != size_t(-1) && m.cells[f.b].type == 4;
        if (!a && !b) continue;
        if (a && b) {
            if (m.cells[f.a].region == m.cells[f.b].region && !rules.kept.count(key)) continue;
            seeds[key] = {f.v, m.cells[f.a].region, m.cells[f.b].region};
            barriers.push_back({f.v[0], f.v[1], f.v[2]});
            for (size_t v : f.v) pins[v] = true;
        } else {
            const auto& c = m.cells[a ? f.a : f.b];
            std::vector<size_t> orient = f.v;
            if (!a) std::reverse(orient.begin(), orient.end());
            seeds[key] = {native::cycle(orient), c.region, 0};
        }
    }
    std::vector<Tet> input;
    for (const auto& c : m.cells)
        if (c.type == 4) input.push_back({c.v[0], c.v[1], c.v[2], c.v[3]});
    wmtk::OptimizerParameters params;
    params.l = length;
    params.l_min = length / 1000;
    params.diag_l = (m.hi - m.lo).norm();
    params.stop_energy = 10;
    params.splitting_l2 = std::pow(1.5 * length, 2);
    params.collapsing_l2 = std::pow((2. / 3) * length, 2);
    params.w_amips = 1;
    params.w_envelope = 0;
    params.skip_good_regions = true;
    Bulk backend(params, m.p, input, pins, m.sizing, barriers);
    backend.local_sizing_splits_only = false;
    backend.cancelled = [&] { return deadline.expired(); };
    backend.set_budget(split_budget, [&] { return deadline.expired(); });
    for (int pass = 0; pass < passes && !deadline.expired(); ++pass) {
        backend.local_operations({0, 1, 1, 1});
        if (!deadline.expired()) backend.local_operations({1, 0, 1, 1});
    }
    backend.validate_fixed_boundary();
    if (!backend.check_mesh_connectivity_validity() || !backend.round_and_check_all_rounded())
        throw std::runtime_error("bulk_tet_mesh_invalid");
    std::vector<V> pp;
    std::vector<int64_t> origin;
    std::vector<Tet> tt;
    std::vector<double> size;
    backend.export_data(pp, origin, tt, &size);
    std::vector<size_t> map(pp.size());
    for (size_t i = 0; i < pp.size(); ++i)
        if (origin[i] >= 0) {
            const size_t v = origin[i];
            if (pins[v] && m.p[v] != pp[i]) throw std::runtime_error("bulk_protected_vertex_moved");
            map[i] = v;
            m.p[v] = pp[i];
            m.sizing[v] = size[i];
        } else
            map[i] = append_volume_vertex(m, pp[i], size[i]);
    std::vector<Cell> proposed;
    proposed.reserve(tt.size());
    for (const auto& t : tt) {
        Cell c{4, 0, {map[t[0]], map[t[1]], map[t[2]], map[t[3]]}};
        if (!std::isfinite(native::quality(m.p, c)))
            throw std::runtime_error("bulk_nonpositive_tet");
        proposed.push_back(std::move(c));
    }
    auto ff = faces_of(proposed);
    std::vector<int> region(proposed.size(), 0);
    std::queue<size_t> q;
    auto seed = [&](size_t i, int r) {
        if (region[i] && region[i] != r) throw std::runtime_error("region_seed_conflict");
        if (!region[i]) {
            region[i] = r;
            q.push(i);
        }
    };
    for (const auto& [key, s] : seeds) {
        auto it = ff.find(key);
        if (it == ff.end()) throw std::runtime_error("material_or_native_boundary_face_lost");
        const auto& f = it->second;
        if (s.r1 == 0) {
            if (f.b != size_t(-1)) throw std::runtime_error("bulk_boundary_became_internal");
            seed(f.a, s.r0);
        } else {
            if (f.b == size_t(-1)) throw std::runtime_error("bulk_material_crack");
            if (f.v == s.oriented) {
                seed(f.a, s.r0);
                seed(f.b, s.r1);
            } else {
                seed(f.a, s.r1);
                seed(f.b, s.r0);
            }
        }
    }
    std::vector<std::vector<size_t>> adjacency(proposed.size());
    for (const auto& [key, f] : ff)
        if (f.b != size_t(-1) && !seeds.count(key)) {
            adjacency[f.a].push_back(f.b);
            adjacency[f.b].push_back(f.a);
        }
    while (!q.empty()) {
        size_t i = q.front();
        q.pop();
        for (size_t j : adjacency[i]) {
            if (region[j] && region[j] != region[i])
                throw std::runtime_error("material_labels_crossed");
            if (!region[j]) {
                region[j] = region[i];
                q.push(j);
            }
        }
    }
    for (size_t i = 0; i < proposed.size(); ++i) {
        if (!region[i]) throw std::runtime_error("unseeded_tet_component");
        proposed[i].region = region[i];
    }
    std::vector<Cell> result;
    result.reserve(proposed.size() + m.cells.size() - input.size());
    for (auto& c : m.cells)
        if (c.type != 4) result.push_back(std::move(c));
    result.insert(
        result.end(),
        std::make_move_iterator(proposed.begin()),
        std::make_move_iterator(proposed.end()));
    m.cells = std::move(result);
    m.rebuild();
    auto after_faces = faces_of(m.cells);
    if (topology(m, after_faces) != original_topology)
        throw std::runtime_error("bulk_material_topology_changed");
    rules.audit(m, after_faces);
    auto report = backend.report();
    report["all_tet_components_processed_together"] = true;
    report["fixed_material_interface_triangles"] = barriers.size();
    report["region_labels_from_exact_interface_flood_fill"] = true;
    report["material_topology_preserved"] = true;
    return report;
}
} // namespace wmtk::components::prismatic_mesh::quality
