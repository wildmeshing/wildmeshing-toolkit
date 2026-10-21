#include "IsotropicRemeshing.h"
#include <wmtk/threading/collector.hpp>
#include <wmtk/threading/indexed_collector.hpp>
#include <wmtk/threading/parallel_for.hpp>

#include <igl/Timer.h>
#include <igl/is_edge_manifold.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include <wmtk/utils/partition_utils.hpp>
#include <wmtk/utils/predicates.hpp>

namespace wmtk::components::isotropic_remeshing {

auto renew = [](auto& m, auto op, auto& tris) {
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, TriMesh::Tuple>>();
    for (auto& e : edges) optup.emplace_back(op, e);
    return optup;
};


auto edge_locker = [](auto& m, const auto& e, int task_id) {
    // TODO: this should not be here
    return m.try_set_edge_mutex_two_ring(e, task_id);
};

IsotropicRemeshing::IsotropicRemeshing(
    std::vector<Eigen::Vector3d> _m_vertex_positions,
    int num_threads,
    bool use_exact)
{
    NUM_THREADS = num_threads;
    m_envelope.use_exact = use_exact;

    m_vertex_group.add(&vertex_attrs);
    m_vertex_group.add(&feature_attrs);
    p_vertex_attrs = &m_vertex_group;

    vertex_attrs.resize(_m_vertex_positions.size());
    feature_attrs.resize(_m_vertex_positions.size());

    for (auto i = 0; i < _m_vertex_positions.size(); i++)
        vertex_attrs[i] = {_m_vertex_positions[i], 0};
}

void IsotropicRemeshing::create_mesh(
    size_t n_vertices,
    const std::vector<std::array<size_t, 3>>& tris,
    const std::vector<size_t>& frozen_verts,
    bool m_freeze,
    double eps,
    double boundary_eps)
{
    wmtk::TriMesh::init(n_vertices, tris);
    std::vector<Eigen::Vector3d> V(n_vertices);
    std::vector<Eigen::Vector3i> F(tris.size());
    for (auto i = 0; i < V.size(); i++) {
        V[i] = vertex_attrs[i].pos;
    }
    for (int i = 0; i < F.size(); ++i) F[i] << (int)tris[i][0], (int)tris[i][1], (int)tris[i][2];
    if (eps > 0) {
        m_envelope.init(V, F, eps);
        m_has_envelope = true;
    } else
        m_envelope.init(V, F, 0.0);

    // TODO: this should not be here
    partition_mesh_morton();

    if (m_freeze) {
        for (auto v : frozen_verts) {
            vertex_attrs[v].freeze = true;
        }
    }
    if (boundary_eps > 0) {
        // Same predicate as the surface envelope; see BoundaryEnvelope::init.
        m_boundary_envelope.init(*this, vertex_attrs, boundary_eps, m_envelope.use_exact);
    } else if (m_freeze) {
        for (auto e : get_edges()) {
            if (is_boundary_edge(e)) {
                vertex_attrs[e.vid(*this)].freeze = true;
                vertex_attrs[e.switch_vertex(*this).vid(*this)].freeze = true;
            }
        }
    }
}

void IsotropicRemeshing::cache_edge_positions(const Tuple& t)
{
    auto& cache = position_cache.local();
    cache.v1 = t.vid(*this);
    cache.v2 = t.switch_vertex(*this).vid(*this);
    cache.v1p = vertex_attrs[cache.v1].pos;
    cache.v2p = vertex_attrs[cache.v2].pos;
    cache.partition_id = vertex_attrs[cache.v1].partition_id;
    cache.feature_edge = is_feature_edge(cache.v1, cache.v2);
    cache.keep_v2_position = false;
}

namespace {
void replace_nbr(std::vector<size_t>& nbrs, const size_t from, const size_t to)
{
    for (size_t& n : nbrs) {
        if (n == from) n = to;
    }
    std::sort(nbrs.begin(), nbrs.end());
    nbrs.erase(std::unique(nbrs.begin(), nbrs.end()), nbrs.end());
}

using wmtk::utils::predicates::faces_along;
} // namespace

Eigen::Vector3d IsotropicRemeshing::area_normal(const std::vector<Tuple>& tris) const
{
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    for (const Tuple& f : tris) {
        const auto vs = oriented_tri_vids(f);
        const auto& p = vertex_attrs;
        n += (p.at(vs[1]).pos - p.at(vs[0]).pos).cross(p.at(vs[2]).pos - p.at(vs[0]).pos);
    }
    return n;
}

size_t IsotropicRemeshing::not_facing(const std::vector<Tuple>& tris, const Eigen::Vector3d& n)
    const
{
    size_t count = 0;
    for (const Tuple& f : tris) {
        const auto vs = oriented_tri_vids(f);
        const auto& p = vertex_attrs;
        if (!faces_along(p.at(vs[0]).pos, p.at(vs[1]).pos, p.at(vs[2]).pos, n)) ++count;
    }
    return count;
}

size_t IsotropicRemeshing::star_folds(const std::vector<Tuple>& tris) const
{
    std::vector<size_t> vs;
    for (const Tuple& f : tris) {
        for (const size_t v : oriented_tri_vids(f)) vs.push_back(v);
    }
    std::sort(vs.begin(), vs.end());
    vs.erase(std::unique(vs.begin(), vs.end()), vs.end());
    size_t count = 0;
    for (const size_t v : vs) {
        const auto star = get_one_ring_tris_for_vertex(tuple_from_vertex(v));
        count += not_facing(star, area_normal(star));
    }
    return count;
}

ReplacedSurface IsotropicRemeshing::replaced(const std::vector<Tuple>& tris) const
{
    ReplacedSurface r;
    r.normal = area_normal(tris);
    r.against = not_facing(tris, r.normal);
    r.star_folds = star_folds(tris);
    return r;
}

bool IsotropicRemeshing::folds(const std::vector<Tuple>& tris, const ReplacedSurface& before)
{
    if (std::max(not_facing(tris, before.normal), not_facing(tris, area_normal(tris))) <=
            before.against &&
        star_folds(tris) <= before.star_folds) {
        return false;
    }
    ++m_refused_inversions;
    return true;
}

void IsotropicRemeshing::set_feature_edges(const std::vector<std::array<size_t, 2>>& edges)
{
    feature_attrs.resize(vertex_attrs.size());
    for (auto& f : feature_attrs.m_attributes) f.nbrs.clear();
    for (const auto& e : edges) {
        if (e[0] == e[1]) continue;
        for (const auto& [a, b] : {std::pair{e[0], e[1]}, std::pair{e[1], e[0]}}) {
            auto& n = feature_attrs.m_attributes[a].nbrs;
            if (std::find(n.begin(), n.end(), b) == n.end()) n.push_back(b);
        }
    }
    m_has_features = !edges.empty();
}

std::vector<std::array<size_t, 2>> IsotropicRemeshing::feature_edges() const
{
    std::vector<std::array<size_t, 2>> out;
    if (!m_has_features) return out;
    for (const auto& t : get_vertices()) {
        const size_t a = t.vid(*this);
        for (const size_t b : feature_attrs.at(a).nbrs) {
            if (a < b) out.push_back({{a, b}});
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

bool IsotropicRemeshing::is_feature_edge(const size_t a, const size_t b) const
{
    if (!m_has_features) return false;
    const auto& n = feature_attrs.at(a).nbrs;
    return std::find(n.begin(), n.end(), b) != n.end();
}

size_t IsotropicRemeshing::feature_degree(const size_t v) const
{
    return m_has_features ? feature_attrs.at(v).nbrs.size() : 0;
}

void IsotropicRemeshing::consolidate_mesh()
{
    if (!m_has_features) {
        TriMesh::consolidate_mesh();
        return;
    }
    // The consolidation numbers the live vertices by rank; the feature edges name vertices, so
    // they follow.
    std::vector<size_t> live;
    for (const auto& t : get_vertices()) live.push_back(t.vid(*this));
    std::sort(live.begin(), live.end());
    std::vector<size_t> rank(vert_capacity(), size_t(-1));
    for (size_t i = 0; i < live.size(); ++i) rank[live[i]] = i;
    TriMesh::consolidate_mesh();
    for (size_t v = 0; v < live.size(); ++v) {
        for (size_t& n : feature_attrs.m_attributes[v].nbrs) n = rank[n];
    }
}

bool IsotropicRemeshing::invariants(const std::vector<Tuple>& new_tris)
{
    // First: it touches only boundary edges, and is far cheaper than the surface test.
    if (!m_boundary_envelope.boundary_edges_inside(*this, vertex_attrs, new_tris)) {
        return false;
    }
    if (m_has_envelope) {
        for (auto& t : new_tris) {
            std::array<Eigen::Vector3d, 3> tris;
            auto vs = oriented_tri_vertices(t);
            for (auto j = 0; j < 3; j++) tris[j] = vertex_attrs[vs[j].vid(*this)].pos;
            if (m_envelope.is_outside(tris)) {
                return false;
            }
        }
    }
    return true;
}

// TODO: this should not be here
void IsotropicRemeshing::partition_mesh()
{
    auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
    for (auto i = 0; i < m_vertex_partition_id.size(); i++)
        vertex_attrs[i].partition_id = m_vertex_partition_id[i];
}

// TODO: morton should not be here, but inside wmtk
void IsotropicRemeshing::partition_mesh_morton()
{
    if (NUM_THREADS == 0) {
        return;
    }
    logger().info("Number of parts: {} by morton", NUM_THREADS);

    std::vector<size_t> partition_id;
    wmtk::partition_vertex_morton(
        vert_capacity(),
        [this](size_t i) { return vertex_attrs[i].pos; },
        NUM_THREADS,
        partition_id);

    for (size_t i = 0; i < partition_id.size(); i++) {
        vertex_attrs[i].partition_id = partition_id[i];
    }
}

std::vector<TriMesh::Tuple> IsotropicRemeshing::new_edges_after(
    const std::vector<TriMesh::Tuple>& tris) const
{
    std::vector<TriMesh::Tuple> new_edges;

    for (auto t : tris) {
        for (auto j = 0; j < 3; j++) {
            new_edges.push_back(tuple_from_edge(t.fid(*this), j));
        }
    }
    wmtk::unique_edge_tuples(*this, new_edges);
    return new_edges;
}

bool IsotropicRemeshing::swap_edge_before(const Tuple& t)
{
    if (!TriMesh::swap_edge_before(t)) return false;
    if (vertex_attrs[t.vid(*this)].freeze && vertex_attrs[t.switch_vertex(*this).vid(*this)].freeze)
        return false;
    const size_t v1 = t.vid(*this), v2 = t.switch_vertex(*this).vid(*this);
    if (is_feature_edge(v1, v2)) return false;
    if (m_has_features) {
        // Nor may it put two edges of a curve into one triangle: the swap joins the apexes a and
        // b, and a curve vertex at an endpoint with curve edges to both would be left with that
        // triangle alone on one side -- flat once smoothing straightens the curve.
        const size_t a = t.switch_edge(*this).switch_vertex(*this).vid(*this);
        const size_t b =
            t.switch_face(*this).value().switch_edge(*this).switch_vertex(*this).vid(*this);
        for (const size_t v : {v1, v2}) {
            if (is_feature_edge(v, a) && is_feature_edge(v, b)) return false;
        }
    }
    if (m_check_inversions) {
        auto& cache = position_cache.local();
        cache.replaced = replaced({t, t.switch_face(*this).value()});
    }
    return true;
}


bool IsotropicRemeshing::swap_edge_after(const TriMesh::Tuple& t)
{
    if (!m_check_inversions) return true;
    const auto& cache = position_cache.local();
    return !folds({t, t.switch_face(*this).value()}, cache.replaced);
}

std::vector<TriMesh::Tuple> IsotropicRemeshing::replace_edges_after_split(
    const std::vector<TriMesh::Tuple>& tris,
    const size_t vid_threshold) const
{
    // For edge split, we do not immediately push back split sub-edges
    // only push back those edges which are already present, but invalidated by hash mechanism.
    std::vector<TriMesh::Tuple> new_edges;
    new_edges.reserve(tris.size());
    for (auto t : tris) {
        auto tmptup = (t.switch_vertex(*this)).switch_edge(*this);
        if (tmptup.vid(*this) < vid_threshold &&
            (tmptup.switch_vertex(*this)).vid(*this) < vid_threshold)
            new_edges.push_back(tmptup);
    }
    return new_edges;
}

std::vector<TriMesh::Tuple> IsotropicRemeshing::new_sub_edges_after_split(
    const std::vector<TriMesh::Tuple>& tris) const
{
    // only push back the renewed original edges
    std::vector<TriMesh::Tuple> new_edges2;
    new_edges2.reserve(tris.size() * 3);
    for (auto t : tris) {
        new_edges2.push_back(t);
        new_edges2.push_back(t.switch_edge(*this));
        new_edges2.push_back((t.switch_vertex(*this)).switch_edge(*this));
    }

    wmtk::unique_edge_tuples(*this, new_edges2);
    return new_edges2;
}


bool IsotropicRemeshing::collapse_edge_before(const Tuple& t)
{
    if (!TriMesh::collapse_edge_before(t)) return false;
    if (vertex_attrs[t.vid(*this)].freeze || vertex_attrs[t.switch_vertex(*this).vid(*this)].freeze)
        return false;
    cache_edge_positions(t);
    // Read on the connectivity before the collapse, for the placement in collapse_edge_after.
    auto& cache = position_cache.local();
    const Tuple t2 = t.switch_vertex(*this);
    cache.v1_input_boundary = vertex_attrs[t.vid(*this)].input_boundary;
    cache.v2_input_boundary = vertex_attrs[t2.vid(*this)].input_boundary;
    cache.v1_on_boundary = wmtk::BoundaryEnvelope::on_input_boundary(*this, vertex_attrs, t);
    cache.v2_on_boundary = wmtk::BoundaryEnvelope::on_input_boundary(*this, vertex_attrs, t2);
    if (m_has_features) {
        cache.keep_v2_position = feature_degree(cache.v2) > 0;
        // No triangle the collapse renames may come out holding two curve edges: the survivor
        // takes over the removed vertex's feature edges, and a survivor a curve split put at
        // its neighbours' midpoint would leave that triangle flat. remove_feature_ears() may:
        // where a curve zigzags, removing one ear makes the next, which its next sweep removes.
        const auto feature_after = [&](size_t x) {
            return is_feature_edge(cache.v1, x) || is_feature_edge(cache.v2, x);
        };
        // Nor may it merge two curves, the boundary counting as one: a triangle the collapse
        // removes whose third vertex has a curve or boundary edge to both ends would leave the
        // two one edge, and the region between them -- that triangle, at least -- gone.
        for (const Tuple& f : get_one_ring_tris_for_vertex(t)) {
            const auto vs = oriented_tri_vids(f);
            if (!std::count(vs.begin(), vs.end(), cache.v2)) continue;
            const size_t fid = f.fid(*this);
            const auto bounding = [&](size_t a, size_t b) {
                return is_feature_edge(a, b) || is_boundary_edge(tuple_from_edge(a, b, fid));
            };
            for (const size_t x : vs) {
                if (x != cache.v1 && x != cache.v2 && bounding(cache.v1, x) &&
                    bounding(cache.v2, x)) {
                    return false;
                }
            }
        }
        for (const Tuple& f : get_one_ring_tris_for_vertex(t)) {
            if (m_removing_ears) break;
            const auto vs = oriented_tri_vids(f);
            if (std::count(vs.begin(), vs.end(), cache.v2)) continue; // removed by the collapse
            size_t x = 0, y = 0;
            for (int k = 0; k < 3; ++k) {
                if (vs[k] == cache.v1) {
                    x = vs[(k + 1) % 3];
                    y = vs[(k + 2) % 3];
                }
            }
            if (int(feature_after(x)) + int(feature_after(y)) + int(is_feature_edge(x, y)) >= 2) {
                return false;
            }
        }
        // A curve vertex goes only along its curve, and only from inside it.
        if (feature_degree(cache.v1) > 0 &&
            (feature_degree(cache.v1) != 2 || !cache.feature_edge)) {
            return false;
        }
        // Nor may a boundary vertex go into a curve vertex inside: the curve would touch the
        // boundary there, pinching the region between them in two.
        if (feature_degree(cache.v2) > 0 && is_boundary_vertex(t) && !is_boundary_vertex(t2)) {
            return false;
        }
    }
    if (m_check_inversions) {
        // Both stars: the triangles the collapse removes, and the ones it renames or moves.
        auto tris = get_one_ring_tris_for_vertex(t);
        for (const Tuple& f : get_one_ring_tris_for_vertex(t2)) {
            const auto vs = oriented_tri_vids(f);
            if (!std::count(vs.begin(), vs.end(), cache.v1)) tris.push_back(f);
        }
        cache.replaced = replaced(tris);
    }
    return true;
}


bool IsotropicRemeshing::collapse_edge_after(const TriMesh::Tuple& t)
{
    const auto& cache = position_cache.local();
    // Onto the midpoint, unless exactly one endpoint is on the input's boundary: then onto that
    // one, as in ShortestEdgeCollapse. The midpoint would pull the outline into the surface,
    // which the boundary envelope refuses, so the collapse would be lost rather than taken.
    // Without a boundary envelope both flags are false and this is the midpoint, as before.
    // A survivor on a feature curve stays where it is, so the curve keeps its shape.
    const Eigen::Vector3d p = cache.keep_v2_position                          ? cache.v2p
                              : cache.v1_on_boundary && !cache.v2_on_boundary ? cache.v1p
                              : cache.v2_on_boundary && !cache.v1_on_boundary
                                  ? cache.v2p
                                  : (cache.v1p + cache.v2p) / 2.0;
    auto vid = t.vid(*this);
    if (m_has_features) {
        // The removed vertex's feature edges go to the survivor.
        assert(vid == cache.v2);
        const std::vector<size_t> removed_nbrs = feature_attrs.at(cache.v1).nbrs;
        auto& kept_nbrs = feature_attrs[vid].nbrs;
        replace_nbr(kept_nbrs, cache.v1, vid);
        kept_nbrs.erase(std::remove(kept_nbrs.begin(), kept_nbrs.end(), vid), kept_nbrs.end());
        for (const size_t x : removed_nbrs) {
            if (x == vid) continue;
            replace_nbr(feature_attrs[x].nbrs, cache.v1, vid);
            if (std::find(kept_nbrs.begin(), kept_nbrs.end(), x) == kept_nbrs.end()) {
                kept_nbrs.push_back(x);
            }
        }
        feature_attrs[cache.v1].nbrs.clear();
    }
    vertex_attrs[vid].pos = p;
    vertex_attrs[vid].partition_id = cache.partition_id;
    vertex_attrs[vid].input_boundary = cache.v1_input_boundary || cache.v2_input_boundary;

    if (m_check_inversions && folds(get_one_ring_tris_for_vertex(t), cache.replaced)) {
        return false;
    }
    return true;
}

bool IsotropicRemeshing::split_edge_before(const Tuple& t)
{
    if (!TriMesh::split_edge_before(t)) return false;
    cache_edge_positions(t);
    // The new vertex continues a boundary edge the tube judges, so it carries the lineage on:
    // otherwise the half between it and an unflagged endpoint would escape the tube. A split
    // of an interior edge puts it inside the surface, where it has nothing to carry.
    auto& cache = position_cache.local();
    cache.split_input_boundary = m_boundary_envelope.initialized() &&
                                 (vertex_attrs[t.vid(*this)].input_boundary ||
                                  vertex_attrs[t.switch_vertex(*this).vid(*this)].input_boundary) &&
                                 is_boundary_edge(t);
    return true;
}


bool IsotropicRemeshing::split_edge_after(const TriMesh::Tuple& t)
{
    const auto& cache = position_cache.local();
    const Eigen::Vector3d p = (cache.v1p + cache.v2p) / 2.0;
    auto vid = t.switch_vertex(*this).vid(*this);
    vertex_attrs[vid].pos = p;
    vertex_attrs[vid].partition_id = cache.partition_id;
    vertex_attrs[vid].input_boundary = cache.split_input_boundary;
    if (m_has_features) {
        // Always written: the slot may hold a vertex that lived there before.
        auto& n = feature_attrs[vid].nbrs;
        n.clear();
        if (cache.feature_edge) {
            n = {cache.v1, cache.v2};
            replace_nbr(feature_attrs[cache.v1].nbrs, cache.v2, vid);
            replace_nbr(feature_attrs[cache.v2].nbrs, cache.v1, vid);
        }
    }
    return true;
}

bool IsotropicRemeshing::smooth_before(const Tuple& t)
{
    if (vertex_attrs[t.vid(*this)].freeze) return false;
    const size_t d = feature_degree(t.vid(*this));
    if (d != 0 && d != 2) return false; // a curve's end, or a junction of curves
    // A curve vertex whose two curve edges share a triangle would flatten it, sliding along the
    // curve. remove_feature_ears() leaves none unless no swap or collapse could remove it.
    if (d == 2 && is_feature_ear_tip(t)) return false;
    return true;
}

bool IsotropicRemeshing::smooth_after(const TriMesh::Tuple& t)
{
    auto one_ring_tris = get_one_ring_tris_for_vertex(t);
    if (one_ring_tris.size() < 2) {
        return false;
    }
    Eigen::Vector3d after_smooth = tangential_smooth(t);
    if (after_smooth.hasNaN()) return false;
    const ReplacedSurface before = m_check_inversions ? replaced(one_ring_tris) : ReplacedSurface();
    vertex_attrs[t.vid(*this)].pos = after_smooth;
    if (m_check_inversions && folds(one_ring_tris, before)) return false;
    return true;
}

bool IsotropicRemeshing::is_feature_ear_tip(const Tuple& v) const
{
    const size_t b = v.vid(*this);
    if (feature_degree(b) != 2) return false;
    const auto& n = feature_attrs.at(b).nbrs;
    for (const Tuple& f : get_one_ring_tris_for_vertex(v)) {
        const auto vs = oriented_tri_vids(f);
        if (std::count(vs.begin(), vs.end(), n[0]) && std::count(vs.begin(), vs.end(), n[1])) {
            return true;
        }
    }
    return false;
}

size_t IsotropicRemeshing::remove_feature_ears()
{
    if (!m_has_features) return 0;
    m_removing_ears = true;
    size_t removed = 0;
    for (const Tuple& face : get_faces()) {
        if (!face.is_valid(*this)) continue; // an earlier step of this sweep changed it
        const size_t f = face.fid(*this);
        const auto vs = oriented_tri_vids(f);
        for (int k = 0; k < 3; ++k) {
            const size_t a = vs[(k + 2) % 3], b = vs[k], c = vs[(k + 1) % 3];
            if (!is_feature_edge(a, b) || !is_feature_edge(b, c)) continue;
            if (is_feature_edge(a, c) || is_boundary_edge(tuple_from_edge(a, c, f))) {
                break; // curves, or the boundary, all round: a region of its own, no ear
            }
            std::vector<Tuple> new_tris;
            // Swap the base (a, c): b gets two triangles on this side. Not where the base is a
            // curve or the boundary, nor where the swap would make a new ear at a or c, nor where
            // it would turn a triangle over.
            const Tuple base = tuple_from_edge(a, c, f);
            const auto across =
                is_feature_edge(a, c) ? std::optional<Tuple>() : base.switch_face(*this);
            bool swappable = across.has_value();
            if (swappable) {
                const auto ws = oriented_tri_vids(*across);
                size_t d = ws[0];
                for (const size_t w : ws) {
                    if (w != a && w != c) d = w;
                }
                const auto P = [this](size_t i) { return vertex_attrs.at(i).pos; };
                const Eigen::Vector3d n_old =
                    (P(b) - P(a)).cross(P(c) - P(a)) + (P(c) - P(a)).cross(P(d) - P(a));
                swappable = !is_feature_edge(a, d) && !is_feature_edge(c, d) &&
                            faces_along(P(a), P(b), P(d), n_old) &&
                            faces_along(P(b), P(c), P(d), n_old);
            }
            if (swappable && swap_edge(base, new_tris)) {
                ++removed;
                break;
            }
            // Else collapse the tip b along the curve, onto the nearer of a and c: the curve
            // loses the corner, and the triangle with it.
            const double la = (vertex_attrs.at(a).pos - vertex_attrs.at(b).pos).norm();
            const double lc = (vertex_attrs.at(c).pos - vertex_attrs.at(b).pos).norm();
            for (const size_t target :
                 la <= lc ? std::array<size_t, 2>{{a, c}} : std::array<size_t, 2>{{c, a}}) {
                if (collapse_edge(tuple_from_edge(b, target, f), new_tris)) {
                    ++removed;
                    break;
                }
            }
            break;
        }
    }
    m_removing_ears = false;
    return removed;
}

double IsotropicRemeshing::compute_edge_cost_collapse(const TriMesh::Tuple& t, double L) const
{
    double l =
        (vertex_attrs[t.vid(*this)].pos - vertex_attrs[t.switch_vertex(*this).vid(*this)].pos)
            .norm();
    if (l < (4. / 5.) * L) return ((4. / 5.) * L - l);
    return -1;
}
double IsotropicRemeshing::compute_edge_cost_split(const TriMesh::Tuple& t, double L) const
{
    double l =
        (vertex_attrs[t.vid(*this)].pos - vertex_attrs[t.switch_vertex(*this).vid(*this)].pos)
            .norm();
    if (l > (4. / 3.) * L) return (l - (4. / 3.) * L);
    return -1;
}

double IsotropicRemeshing::compute_vertex_valence(const TriMesh::Tuple& t) const
{
    std::vector<std::pair<TriMesh::Tuple, size_t>> valences(3);
    valences[0] = std::make_pair(t, get_valence_for_vertex(t));
    auto t2 = t.switch_vertex(*this);
    valences[1] = std::make_pair(t2, get_valence_for_vertex(t2));
    auto t3 = (t.switch_edge(*this)).switch_vertex(*this);
    valences[2] = std::make_pair(t3, get_valence_for_vertex(t3));

    // This is the valence cost of swapping `t`, so it only makes sense for an edge that can
    // be swapped at all -- which means exactly two incident faces. On a non-manifold edge
    // switch_face would hand back one arbitrary member of the fan and the cost would depend
    // on which; swap_edge_before refuses those anyway, so there is nothing to weigh.
    if (is_manifold_edge(t)) {
        auto t4 = (((t.switch_face(*this)).value()).switch_edge(*this)).switch_vertex(*this);
        valences.emplace_back(t4, get_valence_for_vertex(t4));
    }
    double cost_before_swap = 0.0;
    double cost_after_swap = 0.0;

    // check if it's internal vertex or bondary vertex
    // navigating starting one edge and getting back to the start

    for (int i = 0; i < valences.size(); i++) {
        TriMesh::Tuple vert = valences[i].first;
        int val = 6;
        auto one_ring_edges = get_one_ring_edges_for_vertex(vert);
        for (auto edge : one_ring_edges) {
            if (is_boundary_edge(edge)) {
                val = 4;
                break;
            }
        }
        cost_before_swap += (double)(valences[i].second - val) * (valences[i].second - val);
        cost_after_swap +=
            (i < 2) ? (double)(valences[i].second - 1 - val) * (valences[i].second - 1 - val)
                    : (double)(valences[i].second + 1 - val) * (valences[i].second + 1 - val);
    }
    return (cost_before_swap - cost_after_swap);
}

std::vector<double> IsotropicRemeshing::average_len_valen()
{
    double average_len = 0.0;
    double average_valen = 0.0;
    auto edges = get_edges();
    auto verts = get_vertices();
    double maxlen = std::numeric_limits<double>::min();
    double maxval = std::numeric_limits<double>::min();
    double minlen = std::numeric_limits<double>::max();
    double minval = std::numeric_limits<double>::max();
    for (auto& loc : edges) {
        double currentlen = (vertex_attrs[loc.vid(*this)].pos -
                             vertex_attrs[loc.switch_vertex(*this).vid(*this)].pos)
                                .norm();
        average_len += currentlen;
        if (maxlen < currentlen) maxlen = currentlen;
        if (minlen > currentlen) minlen = currentlen;
    }
    average_len /= edges.size();
    for (auto& loc : verts) {
        double currentval = get_one_ring_edges_for_vertex(loc).size();
        average_valen += currentval;
        if (maxval < currentval) maxval = currentval;
        if (minval > currentval) minval = currentval;
    }
    average_valen /= verts.size();
    int cnt = 0;
    std::vector<double> rtn{average_len, maxlen, minlen, average_valen, maxval, minval};
    return rtn;
}

std::vector<TriMesh::Tuple> IsotropicRemeshing::new_edges_after_swap(const TriMesh::Tuple& t) const
{
    // Both edges dereferenced below must have a face on the other side. That holds for the
    // edge a swap just produced -- swap_edge only runs on a manifold edge and leaves the
    // two triangles in place -- and this function has no other valid caller.
    assert(is_manifold_edge(t));

    std::vector<Tuple> new_edges;

    new_edges.push_back(t.switch_edge(*this));
    new_edges.push_back((t.switch_face(*this).value()).switch_edge(*this));
    new_edges.push_back((t.switch_vertex(*this)).switch_edge(*this));
    new_edges.push_back(((t.switch_vertex(*this)).switch_face(*this).value()).switch_edge(*this));
    return new_edges;
}

bool IsotropicRemeshing::collapse_remeshing(double L)
{
    igl::Timer timer;
    timer.start();
    std::vector<std::pair<std::string, Tuple>> collect_all_ops;
    // One slot per (triangle, local edge): for_each_edge only invokes the callback for
    // the triangle that owns the edge, so eid() is unique and no two threads write the
    // same slot -- no lock, and the order is the slot order rather than whichever thread
    // won a mutex.
    {
        auto collect_tuples = threading::indexed_collector<Tuple>(3 * tri_capacity());
        for_each_edge([&](auto& tup) { collect_tuples.set(tup.eid(*this), tup); });
        const auto collected = collect_tuples.compact();
        collect_all_ops.reserve(collected.size());
        for (const auto& t : collected) {
            collect_all_ops.emplace_back("edge_collapse", t);
        }
    }

    wmtk::logger().info(
        "***** collapse get edges time *****: {} ms",
        timer.getElapsedTimeInMilliSec());

    wmtk::logger().info("size for edges to be collapse is {}", collect_all_ops.size());
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = renew;
        executor.priority = [&](auto& m, auto _, auto& e) {
            return m.compute_edge_cost_collapse(e, L);
        };
        executor.num_threads = NUM_THREADS;
        executor.should_renew = [](auto val) { return (val > 0); };
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [val, op, e] = ele;
            if (val < 0) return false; // priority is negated.
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<IsotropicRemeshing>(ExecutionPolicy::kPartition);
        executor.lock_vertices = edge_locker;
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<IsotropicRemeshing>(ExecutionPolicy::kSeq);
        setup_and_execute(executor);
    }

    return true;
}
bool IsotropicRemeshing::split_remeshing(double L)
{
    igl::Timer timer;
    timer.start();
    std::vector<std::pair<std::string, Tuple>> collect_all_ops;
    size_t vid_threshold = 0;
    std::atomic_int count_success = 0;

    {
        auto collect_tuples = threading::indexed_collector<Tuple>(3 * tri_capacity());
        for_each_edge([&](auto& tup) { collect_tuples.set(tup.eid(*this), tup); });
        const auto collected = collect_tuples.compact();
        collect_all_ops.reserve(collected.size());
        for (const auto& t : collected) {
            collect_all_ops.emplace_back("edge_split", t);
        }
    }

    logger().info("***** split get edges time *****: {} ms", timer.getElapsedTimeInMilliSec());

    logger().info("size for edges to be split is {}", collect_all_ops.size());
    wmtk::threading::collector<std::pair<std::string, Tuple>> edges2;
    auto setup_and_execute = [&](auto& executor) {
        vid_threshold = vert_capacity();
        executor.num_threads = NUM_THREADS;
        executor.renew_neighbor_tuples = [&](auto& m, auto op, auto& tris) {
            count_success++;
            auto edges = m.replace_edges_after_split(tris, vid_threshold);
            for (const auto& e2 : m.new_sub_edges_after_split(tris)) {
                edges2.emplace_back(op, e2);
            }
            std::vector<std::pair<std::string, Tuple>> optup;
            for (auto& e : edges) {
                optup.emplace_back(op, e);
            }
            return optup;
        };
        executor.priority = [&](auto& m, auto _, auto& e) {
            return m.compute_edge_cost_split(e, L);
        };
        executor.should_renew = [](auto val) { return (val > 0); };
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [val, op, e] = ele;
            if (val < 0) return false;
            return true;
        };
        // Execute!!
        do {
            count_success.store(0, std::memory_order_release);
            executor(*this, collect_all_ops);
            collect_all_ops.clear();
            for (auto& item : edges2) {
                collect_all_ops.emplace_back(item);
            }
            edges2.clear();
        } while (count_success.load(std::memory_order_acquire) > 0);
    };

    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<IsotropicRemeshing>(ExecutionPolicy::kPartition);
        executor.lock_vertices = edge_locker;
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<IsotropicRemeshing>(ExecutionPolicy::kSeq);
        setup_and_execute(executor);
    }

    return true;
}

bool IsotropicRemeshing::smooth_all_vertices()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("vertex_smooth", loc);

    auto setup_and_execute = [&](auto& executor) {
        executor.num_threads = NUM_THREADS;

        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<IsotropicRemeshing>(ExecutionPolicy::kPartition);
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_vertex_mutex_one_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<IsotropicRemeshing>(ExecutionPolicy::kSeq);
        setup_and_execute(executor);
    }

    return true;
}

bool IsotropicRemeshing::swap_remeshing()
{
    igl::Timer timer;
    timer.start();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    logger().info("size for edges to swap is {}", collect_all_ops.size());

    {
        auto collect_tuples = threading::indexed_collector<Tuple>(3 * tri_capacity());
        for_each_edge([&](auto& tup) { collect_tuples.set(tup.eid(*this), tup); });
        const auto collected = collect_tuples.compact();
        collect_all_ops.reserve(collected.size());
        for (const auto& t : collected) {
            collect_all_ops.emplace_back("edge_swap", t);
        }
    }
    logger().info("***** swap get edges time *****: {} ms", timer.getElapsedTimeInMilliSec());


    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = renew;
        executor.num_threads = NUM_THREADS;
        executor.priority = [](auto& m, auto op, const Tuple& e) {
            return m.compute_vertex_valence(e);
        };
        executor.should_renew = [](auto val) { return (val > 0); };
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [val, _, e] = ele;
            auto val_energy = (m.compute_vertex_valence(e));
            return (val_energy > 1e-5) && ((val_energy - val) * (val_energy - val) < 1e-8);
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<IsotropicRemeshing>(ExecutionPolicy::kPartition);
        executor.lock_vertices = edge_locker;
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<IsotropicRemeshing>(ExecutionPolicy::kSeq);
        setup_and_execute(executor);
    }

    return true;
}
double area(IsotropicRemeshing& m, std::array<TriMesh::Tuple, 3>& verts)
{
    return ((m.vertex_attrs[verts[0].vid(m)].pos - m.vertex_attrs[verts[2].vid(m)].pos)
                .cross(m.vertex_attrs[verts[1].vid(m)].pos - m.vertex_attrs[verts[2].vid(m)].pos))
               .norm() /
           2.0;
}

Eigen::Vector3d normal(IsotropicRemeshing& m, std::array<TriMesh::Tuple, 3>& verts)
{
    return ((m.vertex_attrs[verts[0].vid(m)].pos - m.vertex_attrs[verts[2].vid(m)].pos)
                .cross(m.vertex_attrs[verts[1].vid(m)].pos - m.vertex_attrs[verts[2].vid(m)].pos))
        .normalized();
}

Eigen::Vector3d IsotropicRemeshing::smooth(const TriMesh::Tuple& t)
{
    if (feature_degree(t.vid(*this)) == 2) {
        // Along the curve: the average of the two curve neighbours, as on the boundary.
        const auto& n = feature_attrs.at(t.vid(*this)).nbrs;
        return (vertex_attrs.at(n[0]).pos + vertex_attrs.at(n[1]).pos) / 2;
    }
    auto one_ring_edges = get_one_ring_edges_for_vertex(t);
    if (one_ring_edges.size() < 3) return vertex_attrs[t.vid(*this)].pos;
    Eigen::Vector3d after_smooth(0, 0, 0);
    Eigen::Vector3d after_smooth_boundary(0, 0, 0);
    int boundary = 0;
    for (auto e : one_ring_edges) {
        if (is_boundary_edge(e)) {
            after_smooth_boundary += vertex_attrs[e.vid(*this)].pos;
            boundary++;
            continue;
        }
        after_smooth += vertex_attrs[e.vid(*this)].pos;
    }

    if (boundary)
        after_smooth = after_smooth_boundary / boundary;
    else
        after_smooth /= one_ring_edges.size();
    return after_smooth;
}

Eigen::Vector3d IsotropicRemeshing::tangential_smooth(const Tuple& t)
{
    auto one_ring_tris = get_one_ring_tris_for_vertex(t);
    if (one_ring_tris.size() < 2) return vertex_attrs[t.vid(*this)].pos;
    Eigen::Vector3d after_smooth = smooth(t);
    // get normal and area of each face
    auto area = [](auto& m, auto& verts) {
        return ((m.vertex_attrs[verts[0].vid(m)].pos - m.vertex_attrs[verts[2].vid(m)].pos)
                    .cross(
                        m.vertex_attrs[verts[1].vid(m)].pos - m.vertex_attrs[verts[2].vid(m)].pos))
                   .norm() /
               2.0;
    };
    auto normal = [](auto& m, auto& verts) {
        return ((m.vertex_attrs[verts[0].vid(m)].pos - m.vertex_attrs[verts[2].vid(m)].pos)
                    .cross(
                        m.vertex_attrs[verts[1].vid(m)].pos - m.vertex_attrs[verts[2].vid(m)].pos))
            .normalized();
    };
    auto w0 = 0.0;
    Eigen::Vector3d n0(0.0, 0.0, 0.0);
    for (auto& e : one_ring_tris) {
        auto verts = oriented_tri_vertices(e);
        w0 += area(*this, verts);
        n0 += area(*this, verts) * normal(*this, verts);
    }
    n0 /= w0;
    if (n0.norm() < 1e-10) return vertex_attrs[t.vid(*this)].pos;
    n0 = n0.normalized();
    after_smooth += n0 * n0.transpose() * (vertex_attrs[t.vid(*this)].pos - after_smooth);
    assert(check_mesh_connectivity_validity());
    return after_smooth;
}


bool IsotropicRemeshing::uniform_remeshing(double L, int iterations)
{
    int cnt = 0;
    wmtk::logger().info("target len is: {}", L);

    igl::Timer timer;
    while (cnt < iterations) {
        cnt++;
        wmtk::logger().info("??? Pass ??? {}", cnt);
        // split
        timer.start();
        split_remeshing(L);
        wmtk::logger().info("--------split time-------: {} ms", timer.getElapsedTimeInMilliSec());
        // collpase
        timer.start();
        collapse_remeshing(L);
        wmtk::logger().info(
            "--------collapse time-------: {} ms",
            timer.getElapsedTimeInMilliSec());
        // swap edges
        timer.start();
        swap_remeshing();
        wmtk::logger().info("--------swap time-------: {} ms", timer.getElapsedTimeInMilliSec());
        // feature curves: no triangle may hold two of a curve's edges when smoothing slides it
        if (m_has_features) {
            size_t ears = 0, n;
            do {
                ears += n = remove_feature_ears();
            } while (n > 0 && ears < tri_capacity());
            wmtk::logger().info("removed {} triangles with two feature edges", ears);
        }
        // smoothing
        timer.start();
        smooth_all_vertices();
        wmtk::logger().info("--------smooth time-------: {} ms", timer.getElapsedTimeInMilliSec());

        partition_mesh_morton();

        // Reclaim the slots leaked by this iteration's operations (removed elements
        // are not freed until consolidation). With the preallocated storage this is
        // required so long runs do not exhaust the reserved capacity; it is
        // transparent to the mesh content.
        consolidate_mesh();
    }
    wmtk::logger().info("finished {} remeshing iterations", iterations);
    wmtk::logger().info("+++++++++finished+++++++++");
    return true;
}
bool IsotropicRemeshing::write_triangle_mesh(std::string path)
{
    // write the collapsed mesh into a obj and assert the mesh is manifold
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(vert_capacity(), 3);
    for (auto& t : get_vertices()) {
        auto i = t.vid(*this);
        V.row(i) = vertex_attrs[i].pos;
    }

    Eigen::MatrixXi F = Eigen::MatrixXi::Constant(tri_capacity(), 3, -1);
    for (auto& t : get_faces()) {
        auto i = t.fid(*this);
        auto vs = oriented_tri_vertices(t);
        for (int j = 0; j < 3; j++) {
            F(i, j) = (int)vs[j].vid(*this);
        }
    }
    igl::write_triangle_mesh(path, V, F);
    bool manifold = check_edge_manifold();
    assert(manifold);

    return manifold;
}

} // namespace wmtk::components::isotropic_remeshing