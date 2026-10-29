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
    if (m_cleanup_angle > 0) {
        // For the fold test of cleanup_triangles(): the envelope's BVH finds the input triangles
        // near a triangle, by their index here.
        m_envelope_eps = std::max(eps, 0.);
        m_input_area.resize(F.size());
        for (size_t i = 0; i < F.size(); ++i) {
            m_input_area[i] = (V[F[i][1]] - V[F[i][0]]).cross(V[F[i][2]] - V[F[i][0]]);
        }
    }

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
    cache.feature_id = cache.feature_edge ? feature_edge_id(cache.v1, cache.v2) : -1;
    cache.keep_v2_position = false;
    cache.splitting = false;
}

namespace {
/// Renames the neighbour `from` to `to`, its edge keeping its id. The neighbours stay sorted, each
/// once; where `to` was one already, the first of the two in the list keeps its id.
void replace_nbr(FeatureAttributes& f, const size_t from, const size_t to)
{
    std::vector<std::pair<size_t, int64_t>> e(f.nbrs.size());
    for (size_t k = 0; k < e.size(); ++k) e[k] = {f.nbrs[k] == from ? to : f.nbrs[k], f.ids[k]};
    std::stable_sort(e.begin(), e.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });
    e.erase(
        std::unique(
            e.begin(),
            e.end(),
            [](const auto& a, const auto& b) { return a.first == b.first; }),
        e.end());
    f.nbrs.resize(e.size());
    f.ids.resize(e.size());
    for (size_t k = 0; k < e.size(); ++k) std::tie(f.nbrs[k], f.ids[k]) = e[k];
}

void remove_nbr(FeatureAttributes& f, const size_t v)
{
    for (size_t k = f.nbrs.size(); k-- > 0;) {
        if (f.nbrs[k] != v) continue;
        f.nbrs.erase(f.nbrs.begin() + k);
        f.ids.erase(f.ids.begin() + k);
    }
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

bool IsotropicRemeshing::more_folds(const std::vector<Tuple>& tris, const ReplacedSurface& before)
    const
{
    return std::max(not_facing(tris, before.normal), not_facing(tris, area_normal(tris))) >
               before.against ||
           star_folds(tris) > before.star_folds;
}

bool IsotropicRemeshing::folds(const std::vector<Tuple>& tris, const ReplacedSurface& before)
{
    if (!more_folds(tris, before)) return false;
    ++m_refused_inversions;
    return true;
}

double IsotropicRemeshing::min_angle(const std::vector<Tuple>& tris) const
{
    const auto angle = [](const Eigen::Vector3d& u, const Eigen::Vector3d& v) {
        return std::atan2(u.cross(v).norm(), u.dot(v));
    };
    double a = M_PI;
    for (const Tuple& f : tris) {
        const auto vs = oriented_tri_vids(f);
        for (int k = 0; k < 3; ++k) {
            const Eigen::Vector3d& p = vertex_attrs.at(vs[k]).pos;
            a = std::min(
                a,
                angle(
                    vertex_attrs.at(vs[(k + 1) % 3]).pos - p,
                    vertex_attrs.at(vs[(k + 2) % 3]).pos - p));
        }
    }
    return a * 180 / M_PI;
}

Eigen::Vector3d IsotropicRemeshing::input_normal(const Tuple& f) const
{
    // The input triangles within half the triangle's size, plus the envelope, of its centroid:
    // their summed area vectors are the input's orientation there, which a sliver of it turned
    // over does not change. Of the same face fields only, so that a triangle next to a sharp
    // feature takes the orientation of its own side.
    const auto vs = oriented_tri_vids(f);
    Eigen::Vector3d lo = vertex_attrs.at(vs[0]).pos, hi = lo;
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    double size = 0;
    for (int k = 0; k < 3; ++k) {
        const Eigen::Vector3d& p = vertex_attrs.at(vs[k]).pos;
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
        c += p / 3;
        size = std::max(size, (p - vertex_attrs.at(vs[(k + 1) % 3]).pos).norm());
    }
    const double reach = size / 2 + m_envelope_eps;
    std::vector<int64_t> ids;
    const Eigen::Vector3d grow = Eigen::Vector3d::Constant(reach);
    m_envelope.primitives_in_box(Eigen::Vector3d(lo - grow), Eigen::Vector3d(hi + grow), ids);
    const bool fields = m_has_face_fields && !m_input_values.empty();
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    for (const int64_t i : ids) {
        if (fields && m_input_values[i] != face_field_attrs.at(f.fid(*this)).values) continue;
        if (m_envelope.primitive_distance(i, c) > reach) continue;
        n += m_input_area[i];
    }
    return n;
}

size_t IsotropicRemeshing::input_folds(const std::vector<Tuple>& tris) const
{
    size_t count = 0;
    for (const Tuple& f : tris) {
        const auto vs = oriented_tri_vids(f);
        const auto& p = vertex_attrs;
        const Eigen::Vector3d a =
            (p.at(vs[1]).pos - p.at(vs[0]).pos).cross(p.at(vs[2]).pos - p.at(vs[0]).pos);
        if (a.dot(input_normal(f)) <= 0) ++count;
    }
    return count;
}

bool IsotropicRemeshing::cleanup_allows(
    const std::vector<Tuple>& tris,
    const ReplacedSurface& before,
    const size_t input_folds_before,
    const double floor) const
{
    if (!more_folds(tris, before) && min_angle(tris) > floor) return true;
    // A fold already there confuses the fold check; the input's orientation may then let a
    // repair of it through. Only then: on a thin wall it is noise, and would let folds through.
    return before.folded() && input_folds(tris) < input_folds_before;
}

bool IsotropicRemeshing::cleanup_move(const size_t v, const double floor)
{
    if (stays_put(v) || vertex_attrs[v].freeze) return false;
    const Tuple tv = tuple_from_vertex(v);
    const auto star = get_one_ring_tris_for_vertex(tv);
    if (star.size() < 2) return false;
    const Eigen::Vector3d p0 = vertex_attrs[v].pos;
    std::vector<Eigen::Vector3d> places;
    const size_t d = feature_degree(v);
    if (d == 2) {
        // Along the curve, toward either neighbour on it.
        for (const size_t n : feature_attrs.at(v).nbrs) {
            for (const double s : {0.02, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5}) {
                places.push_back(p0 + s * (vertex_attrs[n].pos - p0));
            }
        }
    } else if (d == 0 && !is_boundary_vertex(tv)) {
        // Within the tangent plane: 24 directions, 6 steps of the star's mean edge length, and
        // the smoothing's own position.
        const Eigen::Vector3d n = area_normal(star);
        if (n.norm() == 0) return false;
        const Eigen::Vector3d u = n.normalized().unitOrthogonal();
        const Eigen::Vector3d w = n.normalized().cross(u);
        const auto ring = get_one_ring_edges_for_vertex(tv);
        double h = 0;
        for (const Tuple& e : ring) h += (vertex_attrs[e.vid(*this)].pos - p0).norm() / ring.size();
        for (int k = 0; k < 24; ++k) {
            const double a = 2 * M_PI * k / 24;
            for (const double s : {0.02, 0.05, 0.1, 0.2, 0.35, 0.5}) {
                places.push_back(p0 + s * h * (std::cos(a) * u + std::sin(a) * w));
            }
        }
        places.push_back(tangential_smooth(tv));
    } else {
        return false;
    }
    // The best places first, by the star's smallest angle; the move itself checks the rest.
    // The trial positions are written outside any operation and put back before one runs.
    std::vector<std::pair<double, size_t>> best;
    for (size_t i = 0; i < places.size(); ++i) {
        vertex_attrs.m_attributes[v].pos = places[i];
        best.emplace_back(-min_angle(star), i);
    }
    vertex_attrs.m_attributes[v].pos = p0;
    std::sort(best.begin(), best.end());
    for (size_t k = 0; k < std::min<size_t>(best.size(), 12); ++k) {
        if (-best[k].first <= floor) break;
        m_smooth_target = places[best[k].second];
        const bool moved = smooth_vertex(tv);
        m_smooth_target.reset();
        if (moved) return true;
    }
    return false;
}

size_t IsotropicRemeshing::cleanup_triangles(const double degrees)
{
    m_cleaning = true;
    size_t repaired = 0;
    std::vector<Tuple> new_tris;
    for (int round = 0; round < 10; ++round) {
        std::vector<std::pair<double, size_t>> bad;
        for (const Tuple& f : get_faces()) {
            const double a = min_angle({f});
            if (a < degrees) bad.emplace_back(a, f.fid(*this));
        }
        std::sort(bad.begin(), bad.end());
        size_t n = 0;
        for (const auto& [_, fid] : bad) {
            const Tuple f = tuple_from_tri(fid);
            // An earlier repair may have removed it, or mended it.
            if (!f.is_valid(*this) || min_angle({f}) >= degrees) continue;
            const auto vs = oriented_tri_vids(f);
            std::vector<Tuple> around;
            for (const size_t v : vs) {
                for (const Tuple& g : get_one_ring_tris_for_vertex(tuple_from_vertex(v))) {
                    if (std::none_of(around.begin(), around.end(), [&](const Tuple& h) {
                            return h.fid(*this) == g.fid(*this);
                        })) {
                        around.push_back(g);
                    }
                }
            }
            std::vector<size_t> nearby;
            for (const Tuple& g : around) {
                for (const size_t w : oriented_tri_vids(g)) {
                    if (std::find(nearby.begin(), nearby.end(), w) == nearby.end())
                        nearby.push_back(w);
                }
            }
            m_cleanup_floor = min_angle(around);
            // Its edges, shortest first; edge k runs from vs[k] to vs[k + 1].
            std::array<std::pair<double, int>, 3> edges;
            for (int k = 0; k < 3; ++k) {
                edges[k] = {
                    (vertex_attrs[vs[k]].pos - vertex_attrs[vs[(k + 1) % 3]].pos).norm(),
                    k};
            }
            std::sort(edges.begin(), edges.end());
            bool done = false;
            {
                const int k = edges[0].second;
                const size_t a = vs[k], b = vs[(k + 1) % 3];
                done = collapse_edge(tuple_from_edge(a, b, fid), new_tris) ||
                       collapse_edge(tuple_from_edge(b, a, fid), new_tris);
            }
            for (int i = 2; i >= 0 && !done; --i) {
                const int k = edges[i].second;
                if (is_feature_edge(vs[k], vs[(k + 1) % 3])) continue;
                done = swap_edge(tuple_from_edge(vs[k], vs[(k + 1) % 3], fid), new_tris);
            }
            // A move of one of its vertices, or else of one around it.
            for (const size_t v : vs) {
                if (done) break;
                done = cleanup_move(v, m_cleanup_floor);
            }
            for (const size_t w : nearby) {
                if (done) break;
                if (std::find(vs.begin(), vs.end(), w) != vs.end()) continue;
                done = cleanup_move(w, m_cleanup_floor);
            }
            if (done) ++n;
        }
        repaired += n;
        if (n == 0) break;
    }
    m_cleanup_floor = 0;
    m_cleaning = false;
    return repaired;
}

void IsotropicRemeshing::set_feature_edges(const std::vector<std::array<size_t, 2>>& edges)
{
    set_feature_edges(edges, std::vector<int64_t>(edges.size(), -1));
}

void IsotropicRemeshing::set_feature_edges(
    const std::vector<std::array<size_t, 2>>& edges,
    const std::vector<int64_t>& ids)
{
    if (ids.size() != edges.size()) {
        log_and_throw_error("set_feature_edges: {} ids for {} edges", ids.size(), edges.size());
    }
    feature_attrs.resize(vertex_attrs.size());
    for (auto& f : feature_attrs.m_attributes) {
        f.nbrs.clear();
        f.ids.clear();
    }
    m_has_edge_ids = false;
    for (size_t i = 0; i < edges.size(); ++i) {
        const auto& e = edges[i];
        if (e[0] == e[1]) continue;
        for (const auto& [a, b] : {std::pair{e[0], e[1]}, std::pair{e[1], e[0]}}) {
            auto& f = feature_attrs.m_attributes[a];
            if (std::find(f.nbrs.begin(), f.nbrs.end(), b) == f.nbrs.end()) {
                f.nbrs.push_back(b);
                f.ids.push_back(ids[i]);
            }
        }
        if (ids[i] != -1) m_has_edge_ids = true;
    }
    m_has_features = !edges.empty();
}

void IsotropicRemeshing::set_fixed_vertices(const std::vector<std::pair<size_t, int64_t>>& vertices)
{
    feature_attrs.resize(vertex_attrs.size());
    for (auto& f : feature_attrs.m_attributes) {
        f.fixed = false;
        f.vertex_id = -1;
    }
    for (const auto& [v, id] : vertices) {
        feature_attrs.m_attributes[v].fixed = true;
        feature_attrs.m_attributes[v].vertex_id = id;
    }
    m_has_fixed = !vertices.empty();
}

void IsotropicRemeshing::set_face_fields(const std::vector<std::vector<double>>& values)
{
    if (values.size() != tri_capacity()) {
        log_and_throw_error(
            "set_face_fields: values for {} faces, the mesh has {}",
            values.size(),
            tri_capacity());
    }
    face_field_attrs.resize(tri_storage_capacity());
    for (size_t f = 0; f < values.size(); ++f) face_field_attrs.m_attributes[f].values = values[f];
    if (m_cleanup_angle > 0) m_input_values = values;
    // From here on the mesh rolls them back, consolidates them and grows them with its faces.
    p_face_attrs = &face_field_attrs;
    m_has_face_fields = true;
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

int64_t IsotropicRemeshing::feature_edge_id(const size_t a, const size_t b) const
{
    if (!m_has_features) return -1;
    const auto& f = feature_attrs.at(a);
    const auto it = std::find(f.nbrs.begin(), f.nbrs.end(), b);
    return it == f.nbrs.end() ? -1 : f.ids[it - f.nbrs.begin()];
}

size_t IsotropicRemeshing::feature_degree(const size_t v) const
{
    return m_has_features ? feature_attrs.at(v).nbrs.size() : 0;
}

bool IsotropicRemeshing::stays_put(const size_t v) const
{
    if (is_fixed(v)) return true;
    const size_t d = feature_degree(v);
    if (d != 0 && d != 2) return true; // a curve's end, or a junction of curves
    // Inside a curve, unless the id changes there: one curve ends and the next starts.
    return d == 2 && m_has_edge_ids && feature_attrs.at(v).ids[0] != feature_attrs.at(v).ids[1];
}

bool IsotropicRemeshing::may_collapse_away(const Tuple& t) const
{
    const size_t v = t.vid(*this);
    if (stays_put(v)) return false;
    // A curve vertex goes only along its curve.
    return feature_degree(v) == 0 || is_feature_edge(v, t.switch_vertex(*this).vid(*this));
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
    // A split leaves the curves where they were; and the sampled test, sampling its halves
    // elsewhere along the edge, could refuse it, leaving a long edge for the pass to work around.
    if (m_has_feature_envelope && !position_cache.local().splitting) {
        // Cheaper than the surface test too: segments, on the curves only. Each once, though
        // both its triangles are new.
        std::vector<std::array<size_t, 2>> checked;
        for (const Tuple& t : new_tris) {
            const auto vs = oriented_tri_vids(t);
            for (int k = 0; k < 3; ++k) {
                const size_t a = std::min(vs[k], vs[(k + 1) % 3]);
                const size_t b = std::max(vs[k], vs[(k + 1) % 3]);
                if (!is_feature_edge(a, b)) continue;
                if (std::find(checked.begin(), checked.end(), std::array<size_t, 2>{{a, b}}) !=
                    checked.end()) {
                    continue;
                }
                checked.push_back({{a, b}});
                if (m_feature_envelope.is_outside(
                        std::array<Eigen::Vector3d, 2>{
                            {vertex_attrs[a].pos, vertex_attrs[b].pos}})) {
                    return false;
                }
            }
        }
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

void IsotropicRemeshing::set_feature_envelope(const double eps)
{
    const auto edges = feature_edges();
    m_has_feature_envelope = eps > 0 && !edges.empty();
    if (!m_has_feature_envelope) return;
    std::vector<Eigen::Vector3d> V(vert_capacity());
    for (size_t i = 0; i < V.size(); ++i) V[i] = vertex_attrs[i].pos;
    std::vector<Eigen::Vector2i> E;
    E.reserve(edges.size());
    for (const auto& e : edges) E.emplace_back(int(e[0]), int(e[1]));
    // Set before init(): an edge envelope builds its exact structure only when it is.
    m_feature_envelope.use_exact = m_envelope.use_exact;
    m_feature_envelope.init(V, E, eps);
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
    position_cache.local().splitting = false;
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
    if (m_check_inversions || m_cleaning) {
        auto& cache = position_cache.local();
        const std::vector<Tuple> tris = {t, t.switch_face(*this).value()};
        cache.replaced = replaced(tris);
        // See cleanup_allows: the input's orientation is read only where there is a fold.
        if (m_cleaning) cache.input_folds = cache.replaced.folded() ? input_folds(tris) : 0;
    }
    return true;
}


bool IsotropicRemeshing::swap_edge_after(const TriMesh::Tuple& t)
{
    if (m_cleaning) {
        // See cleanup_triangles(): the fold check, and where there is a fold the input's
        // orientation.
        const auto& cache = position_cache.local();
        return cleanup_allows(
            {t, t.switch_face(*this).value()},
            cache.replaced,
            cache.input_folds,
            m_cleanup_floor);
    }
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
    if (m_has_features || m_has_fixed) {
        // A curve vertex goes only along its curve, and only from inside it; a fixed one not at
        // all. A survivor on a curve, or fixed, stays where it is.
        if (!may_collapse_away(t)) return false;
        cache.keep_v2_position = feature_degree(cache.v2) > 0 || is_fixed(cache.v2);
        // With a tube around the curves, a collapse along one goes to the midpoint, as any other,
        // unless the survivor stays put: the tube refuses it where that is off the curve.
        if (m_has_feature_envelope && cache.feature_edge && !stays_put(cache.v2)) {
            cache.keep_v2_position = false;
        }
    }
    if (m_has_features) {
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
        // Nor may a boundary vertex go into a curve vertex inside: the curve would touch the
        // boundary there, pinching the region between them in two.
        if (feature_degree(cache.v2) > 0 && is_boundary_vertex(t) && !is_boundary_vertex(t2)) {
            return false;
        }
    }
    if (m_check_inversions || m_cleaning) {
        // Both stars: the triangles the collapse removes, and the ones it renames or moves.
        auto tris = get_one_ring_tris_for_vertex(t);
        for (const Tuple& f : get_one_ring_tris_for_vertex(t2)) {
            const auto vs = oriented_tri_vids(f);
            if (!std::count(vs.begin(), vs.end(), cache.v1)) tris.push_back(f);
        }
        cache.replaced = replaced(tris);
        // See cleanup_allows: the input's orientation is read only where there is a fold.
        if (m_cleaning) cache.input_folds = cache.replaced.folded() ? input_folds(tris) : 0;
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
        // The removed vertex's feature edges go to the survivor, with their ids.
        assert(vid == cache.v2);
        const FeatureAttributes removed = feature_attrs.at(cache.v1);
        auto& kept = feature_attrs[vid];
        replace_nbr(kept, cache.v1, vid);
        remove_nbr(kept, vid);
        for (size_t k = 0; k < removed.nbrs.size(); ++k) {
            const size_t x = removed.nbrs[k];
            if (x == vid) continue;
            replace_nbr(feature_attrs[x], cache.v1, vid);
            if (std::find(kept.nbrs.begin(), kept.nbrs.end(), x) == kept.nbrs.end()) {
                kept.nbrs.push_back(x);
                kept.ids.push_back(removed.ids[k]);
            }
        }
        feature_attrs[cache.v1].nbrs.clear();
        feature_attrs[cache.v1].ids.clear();
    }
    vertex_attrs[vid].pos = p;
    vertex_attrs[vid].partition_id = cache.partition_id;
    vertex_attrs[vid].input_boundary = cache.v1_input_boundary || cache.v2_input_boundary;

    // The repairs' collapses keep the limit whether it is on or not: see cleanup_triangles().
    const auto too_long = [&]() {
        if (!(m_limit_collapse_length || m_cleaning) || m_collapse_target <= 0) return false;
        for (const Tuple& e : get_one_ring_edges_for_vertex(t)) {
            if ((vertex_attrs[e.vid(*this)].pos - p).norm() > 4. / 3. * m_collapse_target) {
                return true;
            }
        }
        return false;
    };
    if (m_cleaning) {
        // See cleanup_triangles(): the length limit, the fold check, and where there is a fold
        // the input's orientation.
        if (too_long()) return false;
        return cleanup_allows(
            get_one_ring_tris_for_vertex(t),
            cache.replaced,
            cache.input_folds,
            m_cleanup_floor);
    }
    if (m_check_inversions && folds(get_one_ring_tris_for_vertex(t), cache.replaced)) {
        return false;
    }
    return !too_long();
}

bool IsotropicRemeshing::split_edge_before(const Tuple& t)
{
    if (!TriMesh::split_edge_before(t)) return false;
    cache_edge_positions(t);
    position_cache.local().splitting = true;
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
    // Write the whole record: the slot can be one consolidate_mesh() freed, which still holds
    // the attributes of the vertex that used to live there -- a frozen one included.
    VertexAttributes& va = vertex_attrs[vid];
    va = VertexAttributes{};
    va.pos = p;
    va.partition_id = cache.partition_id;
    va.input_boundary = cache.split_input_boundary;
    if (m_has_features || m_has_fixed) {
        // Always written: the slot may hold a vertex that lived there before.
        auto& f = feature_attrs[vid];
        f = FeatureAttributes();
        if (cache.feature_edge) {
            // Both halves keep the edge's id.
            f.nbrs = {cache.v1, cache.v2};
            f.ids = {cache.feature_id, cache.feature_id};
            replace_nbr(feature_attrs[cache.v1], cache.v2, vid);
            replace_nbr(feature_attrs[cache.v2], cache.v1, vid);
        }
    }
    if (m_has_face_fields) {
        // Each face at the edge is now two: the half at v1 is the face itself, the half at v2 a
        // new one, which takes its values.
        const auto ring = get_one_ring_tris_for_vertex(t.switch_vertex(*this));
        for (const Tuple& f : ring) {
            const auto vs = oriented_tri_vids(f);
            if (!std::count(vs.begin(), vs.end(), cache.v2)) continue;
            const size_t x = vs[0] != vid && vs[0] != cache.v2   ? vs[0]
                             : vs[1] != vid && vs[1] != cache.v2 ? vs[1]
                                                                 : vs[2];
            for (const Tuple& g : ring) {
                const auto ws = oriented_tri_vids(g);
                if (std::count(ws.begin(), ws.end(), cache.v1) &&
                    std::count(ws.begin(), ws.end(), x)) {
                    face_field_attrs[f.fid(*this)] = face_field_attrs.at(g.fid(*this));
                }
            }
        }
    }
    return true;
}

bool IsotropicRemeshing::smooth_before(const Tuple& t)
{
    position_cache.local().splitting = false;
    if (vertex_attrs[t.vid(*this)].freeze) return false;
    if (stays_put(t.vid(*this))) return false; // see there
    const size_t d = feature_degree(t.vid(*this));
    // A curve vertex whose two curve edges share a triangle would flatten it, sliding along the
    // curve. remove_feature_ears() leaves none unless no swap, collapse or split could remove it.
    if (d == 2 && is_feature_ear_tip(t)) return false;
    return true;
}

bool IsotropicRemeshing::smooth_after(const TriMesh::Tuple& t)
{
    auto one_ring_tris = get_one_ring_tris_for_vertex(t);
    if (one_ring_tris.size() < 2) {
        return false;
    }
    Eigen::Vector3d after_smooth = m_smooth_target ? *m_smooth_target : tangential_smooth(t);
    if (after_smooth.hasNaN()) return false;
    if (m_cleaning) {
        // See cleanup_triangles(): the star's own angle to beat, the fold check, and where there
        // is a fold the input's orientation.
        const ReplacedSurface before = replaced(one_ring_tris);
        const size_t input_folds_before = before.folded() ? input_folds(one_ring_tris) : 0;
        const double floor = min_angle(one_ring_tris);
        vertex_attrs[t.vid(*this)].pos = after_smooth;
        return cleanup_allows(one_ring_tris, before, input_folds_before, floor);
    }
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
            bool collapsed = false;
            for (const size_t target :
                 la <= lc ? std::array<size_t, 2>{{a, c}} : std::array<size_t, 2>{{c, a}}) {
                if (collapse_edge(tuple_from_edge(b, target, f), new_tris)) {
                    ++removed;
                    collapsed = true;
                    break;
                }
            }
            // Else, b inside a curve, split the base at its midpoint: b gets two triangles on
            // this side, the new vertex is on no curve, and nothing moves, so nothing folds. That
            // is where the curve zigzags -- the triangle across the base holds the curve's next
            // edge, so the swap would make the next ear -- and moving b onto a or c turns a
            // triangle over. A junction's corner stays: smoothing does not move a junction.
            if (!collapsed && feature_degree(b) == 2 &&
                split_edge(tuple_from_edge(a, c, f), new_tris)) {
                ++removed;
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

namespace {
/// 4 sqrt(3) area / the sum of the squared edge lengths: 1 for an equilateral triangle, 0 for a
/// flat one.
double shape_quality(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c)
{
    const double s = (b - a).squaredNorm() + (c - b).squaredNorm() + (a - c).squaredNorm();
    return s > 0 ? 2 * std::sqrt(3.) * (b - a).cross(c - a).norm() / s : 0;
}
} // namespace

double IsotropicRemeshing::compute_swap_quality(const TriMesh::Tuple& t) const
{
    // A swap needs exactly two faces at the edge; see compute_vertex_valence.
    if (!is_manifold_edge(t)) return -1;
    const size_t v0 = t.vid(*this), v1 = t.switch_vertex(*this).vid(*this);
    const size_t v2 = t.switch_edge(*this).switch_vertex(*this).vid(*this);
    const size_t v3 =
        t.switch_face(*this).value().switch_edge(*this).switch_vertex(*this).vid(*this);
    const auto q = [this](size_t a, size_t b, size_t c) {
        return std::max(
            shape_quality(vertex_attrs.at(a).pos, vertex_attrs.at(b).pos, vertex_attrs.at(c).pos),
            1e-30);
    };
    // Unsigned: a triangle the swap turns over counts as good, but the fold check refuses it.
    return 1 / std::min(q(v0, v1, v2), q(v0, v1, v3)) - 1 / std::min(q(v0, v2, v3), q(v1, v2, v3));
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
    m_collapse_target = L;
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
        if (m_has_fixed || m_has_edge_ids) {
            // Turned around where only the other end may go: see may_collapse_away.
            executor.edit_operation_maps["edge_collapse"] =
                [](IsotropicRemeshing& m, const Tuple& t) -> std::optional<std::vector<Tuple>> {
                const Tuple u = !m.may_collapse_away(t) && m.may_collapse_away(t.switch_vertex(m))
                                    ? t.switch_vertex(m)
                                    : t;
                std::vector<Tuple> ret;
                if (m.collapse_edge(u, ret)) return ret;
                return {};
            };
        }
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
        // Execute!! Each round splits the halves of the last one's. Where the envelope refuses
        // a split of an edge over twice the threshold, the triangle on it always keeps a side
        // over the threshold, and splitting that one makes the next: the rounds would never run
        // out. The cap is far over the rounds any refinement needs -- each one halves the edges
        // it splits.
        int rounds = 0;
        do {
            if (++rounds > 64) {
                logger().warn("split: stopped after 64 rounds, an edge whose split is refused");
                break;
            }
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
            return m.m_swap_quality ? m.compute_swap_quality(e) : m.compute_vertex_valence(e);
        };
        executor.should_renew = [](auto val) { return (val > 0); };
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [val, _, e] = ele;
            auto val_energy =
                m.m_swap_quality ? m.compute_swap_quality(e) : m.compute_vertex_valence(e);
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
        if (m_cleanup_angle > 0) {
            timer.start();
            const size_t n = cleanup_triangles(m_cleanup_angle);
            wmtk::logger().info(
                "repaired {} triangles under {} degrees in {} ms",
                n,
                m_cleanup_angle,
                timer.getElapsedTimeInMilliSec());
        }

        partition_mesh_morton();

        // Reclaim the slots leaked by this iteration's operations (removed elements
        // are not freed until consolidation). With the preallocated storage this is
        // required so long runs do not exhaust the reserved capacity; it is
        // transparent to the mesh content.
        consolidate_mesh();
    }
    if (m_cleanup_angle > 0) {
        // Once more on the consolidated mesh: a round ends at 10, and where the input is folded
        // the repairs of the last iteration are not done by then.
        const size_t n = cleanup_triangles(m_cleanup_angle);
        wmtk::logger().info("repaired {} triangles under {} degrees", n, m_cleanup_angle);
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