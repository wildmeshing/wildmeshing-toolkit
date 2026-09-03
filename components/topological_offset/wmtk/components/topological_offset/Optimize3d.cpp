#include "TopoOffsetTetMesh.h"

#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SizingField.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <limits>
#include <numeric>
#include <set>
#include <unordered_map>

namespace wmtk::components::topological_offset {

/**
 * The 3D optimization phase, the twin of Optimize2d.cpp. The operations themselves -- split,
 * collapse, the swaps, and the driver that sequences them -- are wmtk::TetOptimizerMesh's. What is
 * here is only what the offset knows: where its two tracked surfaces are, how a vertex on the
 * offset surface is allowed to move, and the single-phase loop that places the front.
 */

namespace {
/// Diagnostic-only running maximum over a smoothing pass, which is run in parallel.
void atomic_max(std::atomic<long long>& target, long long value)
{
    long long cur = target.load();
    while (value > cur && !target.compare_exchange_weak(cur, value)) {
    }
}

/// The three corners of face `fid` in `tid` as (vid, other, other) for a given vid.
inline std::array<size_t, 3> face_corners_from(
    const std::array<size_t, 4>& tet_vids,
    const int skip)
{
    std::array<size_t, 3> f{};
    int k = 0;
    for (int j = 0; j < 4; ++j) {
        if (j != skip) f[size_t(k++)] = tet_vids[size_t(j)];
    }
    return f;
}
} // namespace

// ---------------------------------------------------------------------------------------------
// The two tracked surfaces, live.
// ---------------------------------------------------------------------------------------------

bool TopoOffsetTetMesh::face_is_offset_surface_live(const Tuple& f) const
{
    const size_t ta = f.tid(*this);
    const std::optional<Tuple> opp = f.switch_tetrahedron(*this);
    if (!opp) {
        // Domain boundary. A band cell here means the band was clipped by the bounding box, and
        // that face is offset surface -- its vertices can never reach the target distance, which
        // is precisely the thing that must be measured rather than hidden.
        return cell_is_offset_band(ta);
    }
    const size_t tb = opp->tid(*this);
    const bool a = cell_is_offset_band(ta), b = cell_is_offset_band(tb);
    if (a == b) return false; // both in the band, or neither: not the band's surface
    // The band's inner interface, against the input complex it wraps, sits at distance 0 by
    // construction and would drag the reported error to target_distance everywhere.
    return !cell_is_input_complex(a ? tb : ta);
}

bool TopoOffsetTetMesh::edge_is_offset_surface_live(const size_t a, const size_t b) const
{
    for (const size_t tid : get_incident_tids_for_edge(a, b)) {
        const auto vs = oriented_tet_vids(tid);
        for (const size_t c : vs) {
            if (c == a || c == b) continue;
            const auto found = try_tuple_from_face({{a, b, c}});
            if (found && face_is_offset_surface_live(std::get<0>(*found))) return true;
        }
    }
    return false;
}

std::vector<std::array<size_t, 2>> TopoOffsetTetMesh::offset_surface_edges() const
{
    std::set<std::array<size_t, 2>> edges;
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        const auto vs = get_face_vids(f);
        for (int i = 0; i < 3; ++i) {
            std::array<size_t, 2> e{{vs[i], vs[(i + 1) % 3]}};
            if (e[0] > e[1]) std::swap(e[0], e[1]);
            edges.insert(e);
        }
    }
    return std::vector<std::array<size_t, 2>>(edges.begin(), edges.end());
}

std::vector<TopoOffsetTetMesh::Tuple> TopoOffsetTetMesh::offset_surface_faces_live_at(
    const size_t vid) const
{
    std::vector<Tuple> result;
    std::set<size_t> seen;
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        const auto tv = oriented_tet_vids(tid);
        for (int skip = 0; skip < 4; ++skip) {
            if (tv[size_t(skip)] == vid) continue;
            const auto [ft, fid] = tuple_from_face(face_corners_from(tv, skip));
            if (!seen.insert(fid).second) continue;
            if (face_is_offset_surface_live(ft)) result.push_back(ft);
        }
    }
    return result;
}

const OffsetPotential3D& TopoOffsetTetMesh::potential_for_face(const Tuple& f) const
{
    const size_t ta = f.tid(*this);
    const std::optional<Tuple> opp = f.switch_tetrahedron(*this);
    size_t band = ta;
    if (!cell_is_offset_band(ta) && opp && cell_is_offset_band(opp->tid(*this)))
        band = opp->tid(*this);
    return potential_for_region(band < m_cell_region.size() ? m_cell_region[band] : -1);
}

void TopoOffsetTetMesh::label_offset_boundary()
{
    // Runs once at the top of the optimization, never again -- as in 2D. It only upgrades the
    // offset surface to its own class: region and input faces keep class 0 and are
    // envelope-checked by the shared operations exactly as in tetwild and simwild.

    // Cell quality, which the shared operations read and keep up to date from here on.
    for (const Tuple& t : get_tets()) {
        m_tet_attribute[t.tid(*this)].m_quality = get_quality(t);
    }

    size_t n_off = 0, n_wall_band = 0;
    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        if (m_face_extra[fid].label != 2) {
            continue; // face not on the offset, skip
        }
        const std::optional<Tuple> opp = f.switch_tetrahedron(*this);
        if (!opp) {
            // The band ran into the domain wall. With no second tet this cannot be classified
            // as offset surface -- and must not be, or the wall face would lose the per-tag
            // containment that keeps the box a box.
            ++n_wall_band;
            continue;
        }
        if (m_tet_attribute[f.tid(*this)].label == m_tet_attribute[opp->tid(*this)].label) {
            continue; // face not between different labels, skip
        }

        m_face_attribute[fid].m_is_surface_fs = true;
        m_face_attribute[fid].m_surface_class = OFFSET_SURFACE_CLASS;
        ++n_off;

        for (const size_t vid : get_face_vids(f)) {
            m_vertex_extra[vid].m_is_on_offset = true;
            // The base's union flag, and the one the shared operations actually read.
            m_vertex_attribute[vid].m_is_on_surface = true;
        }
    }

    size_t n_reg = 0, n_box = 0;
    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        n_reg += face_is_region(fid);
        n_box += (m_face_attribute[fid].m_is_bbox_fs >= 0);
    }
    logger().info(
        "\ttracked faces: {} offset surface, {} region boundary (input complex included), {} "
        "bbox | {} band faces lying ON the wall",
        n_off,
        n_reg,
        n_box,
        n_wall_band);
}

void TopoOffsetTetMesh::warn_if_offset_reaches_domain_boundary() const
{
    // A band face with no opposite tet lies ON the domain boundary: the band ran out of room
    // before reaching target_distance. Counted in vertices as well as faces because the vertices
    // are what is pinned.
    size_t n_faces = 0, n_verts = 0;
    std::vector<bool> counted(vert_capacity(), false);
    for (const Tuple& f : get_faces()) {
        if (f.switch_tetrahedron(*this)) continue; // interior face; the band has room here
        if (!cell_is_offset_band(f.tid(*this))) continue;
        ++n_faces;
        for (const size_t v : get_face_vids(f)) {
            if (!counted[v]) {
                counted[v] = true;
                ++n_verts;
            }
        }
    }
    if (n_faces == 0) return;

    logger().warn(
        "Offset band reaches the domain boundary: {} band faces ({} vertices) lie ON the "
        "bounding box. target_distance ({}) exceeds the clearance between the input complex and "
        "the box, so the offset is CLIPPED there and cannot reach the target distance -- those "
        "vertices are on the frozen bounding box and no operation may move them. They ARE "
        "included in max_dist_err / avg_dist_err (see compute_distance_deviation), so expect the "
        "reported error to be dominated by them and to stay flat across iterations. Reduce "
        "target_distance, or pad the background mesh.",
        n_faces,
        n_verts,
        m_offset_params.target_distance);
}

// ---------------------------------------------------------------------------------------------
// Containment.
// ---------------------------------------------------------------------------------------------

std::shared_ptr<SampleEnvelope> TopoOffsetTetMesh::containment_for(
    const uint64_t region_mask,
    const bool on_offset) const
{
    // The region side first, and OUTSIDE the lock: envelope_for_mask() takes m_isect_mutex
    // itself and std::mutex is not recursive.
    const std::shared_ptr<SampleEnvelope> region = envelope_for_mask(region_mask);

    // The offset side, Phase A only: the placing phases have to move the surface, and a tube
    // around where it currently sits would cap how far it can travel. Null before the first
    // rebuild.
    const bool hold_offset = on_offset && m_phase == OptPhase::A && m_offset_envelope != nullptr;

    if (!hold_offset) return region; // may itself be null: nothing contains this simplex
    if (!region) return m_offset_envelope;

    // On both: the intersection, inside every tube it lies on. Memoized per region mask;
    // rebuild_offset_envelope() clears the map, so an entry can never outlive its tube.
    {
        std::lock_guard<std::mutex> lock(m_isect_mutex);
        const auto it = m_offset_isect_cache.find(region_mask);
        if (it != m_offset_isect_cache.end()) return it->second;
    }
    std::shared_ptr<SampleEnvelope> isect = std::make_shared<IntersectionEnvelope>(
        std::vector<std::shared_ptr<SampleEnvelope>>{region, m_offset_envelope});
    std::lock_guard<std::mutex> lock(m_isect_mutex);
    return m_offset_isect_cache.emplace(region_mask, std::move(isect)).first->second;
}

bool TopoOffsetTetMesh::project_into_containment(const size_t vid, Vector3d& x) const
{
    const uint64_t mask = vertex_boundary_mask(vid);
    if (mask == 0) return true; // nothing holds this vertex; any position is valid

    // The real members, never envelope_for_mask()'s composite -- see TagEnvelopes.hpp.
    std::vector<const SampleEnvelope*> members;
    for (const auto& [tag, env] : m_tag_envelopes) {
        const auto it = m_tag_bit.find(tag);
        if (it != m_tag_bit.end() && (mask & (uint64_t(1) << it->second))) {
            members.push_back(env.get());
        }
    }
    if (members.empty()) return true; // every bit dangled: no tube was ever built for them

    // Alternating projection, worst violation first.
    constexpr int kMaxRounds = 8;
    for (int round = 0; round < kMaxRounds; ++round) {
        const SampleEnvelope* worst = nullptr;
        double worst_d2 = -1.;
        for (const SampleEnvelope* e : members) {
            if (!e->is_outside(x)) continue;
            const double d2 = e->squared_distance(x);
            if (d2 > worst_d2) {
                worst_d2 = d2;
                worst = e;
            }
        }
        if (!worst) return true; // inside every tube it lies on
        Vector3d proj = x;
        worst->nearest_point(x, proj);
        if (!proj.allFinite()) return false;
        x = proj;
    }
    for (const SampleEnvelope* e : members) {
        if (e->is_outside(x)) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------------------------
// deform_others: other input regions deform instead of being envelope-held.
// ---------------------------------------------------------------------------------------------

bool TopoOffsetTetMesh::cell_is_deformable(const size_t tid) const
{
    if (m_deform_tags.empty()) return false;
    if (m_tet_attribute[tid].label != 0) return false;
    const auto& tags = m_tet_attribute[tid].tag;
    if (tags.empty()) return false;
    for (const int64_t t : tags) {
        if (m_deform_tags.count(t) == 0) return false;
    }
    return true;
}

bool TopoOffsetTetMesh::cell_is_released_band(const size_t tid) const
{
    // A band cell that is released material: every tag besides the offset output tag belongs to
    // a released object, and there is at least one such tag. Read only by the front placement
    // objective and the rest stamping. As in 2D.
    if (m_deform_tags.empty()) return false;
    if (m_tet_attribute[tid].label != 2) return false;
    bool has_released = false;
    for (const int64_t t : m_tet_attribute[tid].tag) {
        if (m_offset_output_tag_ids.count(t)) continue;
        if (m_deform_tags.count(t) == 0) return false;
        has_released = true;
    }
    return has_released;
}

void TopoOffsetTetMesh::stamp_rest_cell(const size_t tid)
{
    if (!cell_is_plastic(tid) && !cell_is_deformable(tid) && !cell_is_released_band(tid)) return;
    const auto vs = oriented_tet_vids(tid);
    TetAttributes& x = m_tet_attribute[tid];
    for (int i = 0; i < 4; ++i) x.rest_pos[size_t(i)] = m_vertex_attribute[vs[size_t(i)]].m_posf;
    x.rest_valid = true;
}

void TopoOffsetTetMesh::stamp_plastic_rests()
{
    if (!m_plastic_active) return;
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (!cell_is_plastic(tid) && !cell_is_released_band(tid)) continue;
        const auto vs = oriented_tet_vids(tid);
        TetAttributes& x = m_tet_attribute[tid];
        for (int i = 0; i < 4; ++i) {
            x.rest_pos[size_t(i)] = m_vertex_attribute[vs[size_t(i)]].m_posf;
        }
        x.rest_valid = true;
    }
}

namespace {
/// The rest-shape cell of tet `tid` at the moving vertex `vid`, with the same corner
/// permutation the shared smoother applies (vid first, winding preserved). False when the rest
/// is degenerate or inverted -- a rest that holds no shape to preserve.
template <typename Mesh>
bool rest_cell_at(const Mesh& m, const size_t tid, const size_t vid, RestAMIPSEnergy3D::Cell& c)
{
    const auto& ta = m.m_tet_attribute[tid];
    if (!ta.rest_valid) return false;
    const std::array<size_t, 4> orig = m.oriented_tet_vids(tid);
    const std::array<size_t, 4> vs = wmtk::orient_preserve_tet_reorder(orig, vid);
    std::array<int, 4> from{};
    for (int k = 0; k < 4; ++k) {
        for (int j = 0; j < 4; ++j) {
            if (orig[size_t(j)] == vs[size_t(k)]) from[size_t(k)] = j;
        }
    }
    Eigen::Matrix3d R;
    for (int k = 1; k < 4; ++k) {
        R.col(k - 1) = ta.rest_pos[size_t(from[size_t(k)])] - ta.rest_pos[size_t(from[0])];
    }
    if (!(R.determinant() > 0.)) return false;
    c.q1 = m.m_vertex_attribute[vs[1]].m_posf;
    c.q2 = m.m_vertex_attribute[vs[2]].m_posf;
    c.q3 = m.m_vertex_attribute[vs[3]].m_posf;
    c.rest_inv = R.inverse();
    return true;
}
} // namespace

bool TopoOffsetTetMesh::smooth_plastic_vertex(const Tuple& t)
{
    // The plastic medium's smoothing: rest-shape AMIPS over the one-ring and nothing else. Rest
    // is the shape at the group's start (stamp_plastic_rests), so the term resists only the
    // increment. No regular-tet term, no quality veto. Accept on exact inversion of the ring.
    const size_t vid = t.vid(*this);
    const std::vector<Tuple> ring = get_one_ring_tets_for_vertex(t);
    for (const Tuple& loc : ring) {
        if (is_inverted_f(loc)) {
            ++m_smooth_rejects.already_inverted;
            return false;
        }
    }
    std::vector<RestAMIPSEnergy3D::Cell> cells;
    for (const Tuple& loc : ring) {
        const size_t tid = loc.tid(*this);
        if (!cell_is_plastic(tid)) continue;
        RestAMIPSEnergy3D::Cell c;
        if (rest_cell_at(*this, tid, vid, c)) cells.push_back(c);
    }
    if (cells.empty()) return false;
    auto energy = std::make_shared<RestAMIPSEnergy3D>(std::move(cells), 1.0);
    auto& solver = m_solver.local();
    if (!solver) {
        solver = polysolve::nonlinear::Solver::create(
            optimization::basic_nonlinear_solver_params,
            optimization::basic_linear_solver_params,
            1,
            opt_logger());
    }
    const Vector3d x0 = m_vertex_attribute[vid].m_posf;
    Eigen::VectorXd x = x0;
    try {
        solver->minimize(*energy, x);
    } catch (const std::exception&) {
    }
    set_vertex_position(vid, Vector3d(x));
    for (const Tuple& loc : ring) {
        if (is_inverted(loc)) {
            set_vertex_position(vid, x0);
            ++m_smooth_rejects.inverted;
            return false;
        }
    }
    for (const Tuple& loc : ring) set_cell_quality(loc.tid(*this), get_quality(loc));
    ++m_smooth_rejects.accepted;
    m_released_tube_dirty.store(true, std::memory_order_release); // the boundary may have moved
    return true;
}

void TopoOffsetTetMesh::release_deformable_regions()
{
    // Held: any tag on an input-complex cell or on a wall face, and the sheet group. Everything
    // else with an envelope is an object the offset merely shares the scene with, and
    // deform_others releases it. Never released, whatever else it touches: every tag the
    // offset_selection expression names. As in 2D.
    std::set<int64_t> kept;
    std::set<int64_t> source_tags;
    if (m_offset_params.offset_selection) {
        for (const int64_t t : m_offset_params.offset_selection->tags_involved()) {
            kept.insert(t);
            source_tags.insert(t);
        }
    }
    for (const Tuple& f : get_faces()) {
        if (f.switch_tetrahedron(*this)) continue; // wall faces only
        for (const int64_t t : m_tet_attribute[f.tid(*this)].tag) kept.insert(t);
    }
    if (m_sheet_tag >= 0) kept.insert(m_sheet_tag);

    m_deform_tags.clear();
    for (const auto& [tag, env] : m_tag_envelopes) {
        if (kept.count(tag) == 0) m_deform_tags.insert(tag);
    }
    m_source_tags = source_tags;
    if (m_deform_tags.empty()) {
        logger().info("[deform_others] nothing to release: every tagged region is held");
        return;
    }

    std::string released;
    for (const int64_t t : m_deform_tags) {
        released += " " + m_tag_id_to_name.at(t);
    }

    // Edit the masks in place, never recompute them: reduce a freed vertex's mask to its source
    // bits. Never freed: a face that also borders a source tag, a face both of whose tets carry
    // a source tag (complex-internal), or a wall vertex.
    uint64_t released_bits = 0;
    for (const int64_t t : m_deform_tags) {
        const auto it = m_tag_bit.find(t);
        if (it != m_tag_bit.end()) released_bits |= (uint64_t(1) << it->second);
    }
    if (released_bits) {
        uint64_t source_bits = 0;
        for (const int64_t t : source_tags) {
            const auto it = m_tag_bit.find(t);
            if (it != m_tag_bit.end()) source_bits |= (uint64_t(1) << it->second);
        }
        const auto tet_has_source = [&](const size_t tid) {
            for (const int64_t t : m_tet_attribute[tid].tag) {
                if (source_tags.count(t)) return true;
            }
            return false;
        };
        size_t n_freed = 0;
        for (const Tuple& f : get_faces()) {
            const size_t fid = f.fid(*this);
            if (!m_face_attribute[fid].m_is_surface_fs) continue;
            const std::optional<Tuple> t_opp = f.switch_tetrahedron(*this);
            if (!t_opp) continue; // the domain wall
            CellTag face_tags;
            const auto& t0 = m_tet_attribute[f.tid(*this)].tag;
            const auto& t1 = m_tet_attribute[t_opp->tid(*this)].tag;
            std::set_symmetric_difference(
                t0.begin(),
                t0.end(),
                t1.begin(),
                t1.end(),
                std::inserter(face_tags, face_tags.begin()));
            bool released_here = false, touches_source = false;
            for (const int64_t t : face_tags) {
                if (m_deform_tags.count(t)) released_here = true;
                if (source_tags.count(t)) touches_source = true;
            }
            if (!released_here || touches_source) continue;
            if (tet_has_source(f.tid(*this)) && tet_has_source(t_opp->tid(*this))) {
                continue; // complex-internal: anchored, never freed
            }
            for (const size_t v : get_face_vids(f)) {
                if (!m_vertex_attribute[v].on_bbox_faces.empty()) continue; // wall stays held
                const uint64_t kept_bits = m_vertex_extra[v].m_boundary_mask & source_bits;
                if (m_vertex_extra[v].m_boundary_mask != kept_bits) ++n_freed;
                m_vertex_extra[v].m_boundary_mask = kept_bits;
            }
        }
        logger().info(
            "[deform_others] {} boundary vertices freed down to their source bits",
            n_freed);
    }

    size_t n_cells = 0;
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (cell_is_deformable(tid)) {
            stamp_rest_cell(tid);
            ++n_cells;
        }
    }
    logger().info(
        "[deform_others] released:{} | {} deformable cells stamped with their rest shape; "
        "held: {} envelopes",
        released,
        n_cells,
        m_tag_envelopes.size());
    m_released_tube_dirty.store(true, std::memory_order_release);
    released_envelope(); // built here, at a consistent moment, not at some mid-pass first query
}

std::shared_ptr<polysolve::nonlinear::Problem> TopoOffsetTetMesh::rest_energy_for_vertex(
    const size_t vid) const
{
    if (m_deform_tags.empty()) return nullptr;
    std::vector<RestAMIPSEnergy3D::Cell> cells;
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        // Released-band cells too: a released object's boundary inside the band has band-labeled
        // ring cells, which cell_is_deformable() skips.
        if (!cell_is_deformable(tid) && !cell_is_released_band(tid)) continue;
        RestAMIPSEnergy3D::Cell c;
        if (rest_cell_at(*this, tid, vid, c)) cells.push_back(c);
    }
    if (cells.empty()) return nullptr;
    // The shared smoother's own AMIPS factor, so the rest term and the regular-tet quality term
    // it sums with sit at 1:1.
    const double w = m_params.w_amips > 0 ? m_s_amips * m_params.w_amips : 1.0;
    return std::make_shared<RestAMIPSEnergy3D>(std::move(cells), w);
}

bool TopoOffsetTetMesh::face_borders_released_boundary(const Tuple& f) const
{
    // The same test release_deformable_regions() freed vertices by: the incident tets' CURRENT
    // tag symmetric difference contains a released tag and no source tag.
    const std::optional<Tuple> opp = f.switch_tetrahedron(*this);
    if (!opp) return false; // the domain wall
    CellTag face_tags;
    const auto& t0 = m_tet_attribute[f.tid(*this)].tag;
    const auto& t1 = m_tet_attribute[opp->tid(*this)].tag;
    std::set_symmetric_difference(
        t0.begin(),
        t0.end(),
        t1.begin(),
        t1.end(),
        std::inserter(face_tags, face_tags.begin()));
    bool released_here = false;
    for (const int64_t t : face_tags) {
        if (m_source_tags.count(t)) return false;
        if (m_deform_tags.count(t)) released_here = true;
    }
    return released_here;
}

std::shared_ptr<SampleEnvelope> TopoOffsetTetMesh::released_envelope() const
{
    // deform_others' ops-only tube: a tube around the CURRENT released boundaries, consulted by
    // surface_envelope_for_face() -- the dispatch every operation containment check comes
    // through and no smoothing path does. Lazy, on a dirty flag the smoothing accepts set; never
    // rebuilt mid-operation. See the 2D twin.
    if (m_deform_tags.empty()) return nullptr;
    std::lock_guard<std::mutex> lock(m_released_mutex);
    if (!m_released_tube_dirty.load(std::memory_order_acquire)) return m_released_envelope;
    if (const_cast<TopoOffsetTetMesh*>(this)->m_vertex_attribute.recording.local()) {
        return m_released_envelope;
    }
    std::vector<Eigen::Vector3i> tris;
    for (const Tuple& f : get_faces()) {
        if (!m_face_attribute[f.fid(*this)].m_is_surface_fs) continue;
        if (!face_borders_released_boundary(f)) continue;
        const auto vs = get_face_vids(f);
        tris.emplace_back(int(vs[0]), int(vs[1]), int(vs[2]));
    }
    if (tris.empty()) {
        m_released_envelope = nullptr;
    } else {
        std::vector<Eigen::Vector3d> verts(vert_capacity());
        for (size_t i = 0; i < vert_capacity(); ++i) {
            verts[i] = m_vertex_attribute[i].m_posf;
        }
        const double eps =
            std::max(m_offset_params.offset_envelope_rel * m_offset_params.target_distance, 1e-12);
        m_released_envelope = std::make_shared<SampleEnvelope>(/*exact=*/true);
        m_released_envelope->init(verts, tris, eps);
    }
    m_released_tube_dirty.store(false, std::memory_order_release);
    return m_released_envelope;
}

// ---------------------------------------------------------------------------------------------
// Normals and gradients at the front.
// ---------------------------------------------------------------------------------------------

Vector3d TopoOffsetTetMesh::offset_vertex_normal(const size_t vid) const
{
    // See the declaration: the direction the offset grew along, taken from the geometry rather
    // than the mesh. Flips discontinuously across the medial axis.
    if (m_input_complex_bvh) {
        const Vector3d x = m_vertex_attribute[vid].m_posf;
        const Vector3d foot = m_input_complex_bvh->nearest_point(x);
        const Vector3d d = x - foot;
        const double len = d.norm();
        if (len > 0.) return d / len;
    }
    return Vector3d::Zero();
}

double TopoOffsetTetMesh::front_vertex_normal_gradient(const size_t vid) const
{
    // ||grad F|| at the vertex's current position, F the objective smooth_front_vertex_phase_b()
    // minimises, along the move direction under front_normal_projection.
    const Vector3d x = m_vertex_attribute[vid].m_posf;
    Eigen::VectorXd xv = x, g(3);
    phase_b_front_objective(vid, x)->gradient(xv, g);
    if (!g.allFinite()) return std::numeric_limits<double>::infinity();
    const Vector3d n = front_vertex_move_direction(vid);
    if (n.squaredNorm() > 0.) return std::abs(n.dot(Vector3d(g)));
    return g.norm();
}

// ---------------------------------------------------------------------------------------------
// Diagnostics: containment audit, mask health, smoothing trace.
// ---------------------------------------------------------------------------------------------

void TopoOffsetTetMesh::audit_surface_containment(const std::string& when) const
{
    struct Bad
    {
        std::array<size_t, 3> v{{0, 0, 0}};
        uint64_t mask = 0;
        bool offset_class = false;
        double worst_d = 0.; ///< furthest sample distance to a real member tube
        double worst_end_d = 0.; ///< furthest CORNER distance -- 0 means every corner is inside
        double len = 0.;
        int worst_tag = -1;
    };
    std::vector<Bad> bad;
    size_t n_tracked = 0, n_offset_class = 0, n_region_class = 0, n_other = 0;
    size_t bad_offset = 0, bad_region = 0, bad_other = 0;

    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        if (!m_face_attribute[fid].m_is_surface_fs) continue;
        ++n_tracked;
        const std::array<size_t, 3> vids = get_face_vids(f);
        const bool is_offset = face_is_offset(fid);
        const uint64_t mask = is_offset ? uint64_t(0) : face_mask(vids);
        if (is_offset)
            ++n_offset_class;
        else if (mask != 0)
            ++n_region_class;
        else
            ++n_other;

        // Exactly the dispatch the sanity check uses, so this cannot disagree with it.
        if (!surface_triangle_is_outside(vids[0], vids[1], vids[2])) continue;

        Bad r;
        r.v = vids;
        r.mask = mask;
        r.offset_class = is_offset;
        const Vector3d& pa = m_vertex_attribute[vids[0]].m_posf;
        const Vector3d& pb = m_vertex_attribute[vids[1]].m_posf;
        const Vector3d& pc = m_vertex_attribute[vids[2]].m_posf;
        r.len = std::max({(pb - pa).norm(), (pc - pb).norm(), (pa - pc).norm()});
        if (r.offset_class)
            ++bad_offset;
        else if (mask != 0)
            ++bad_region;
        else
            ++bad_other;

        // How far outside, per real member -- never the composite. Sampled over the triangle.
        const auto probe = [&](const std::shared_ptr<SampleEnvelope>& env, int tag) {
            if (!env) return;
            for (const size_t v : vids) {
                const double d =
                    std::sqrt(std::max(env->squared_distance(m_vertex_attribute[v].m_posf), 0.));
                if (d > r.worst_end_d) r.worst_end_d = d;
            }
            constexpr int kSamples = 6;
            for (int i = 0; i <= kSamples; ++i) {
                for (int j = 0; j <= kSamples - i; ++j) {
                    const double u = double(i) / kSamples, w = double(j) / kSamples;
                    const Vector3d q = pa + u * (pb - pa) + w * (pc - pa);
                    const double d = std::sqrt(std::max(env->squared_distance(q), 0.));
                    if (d > r.worst_d) {
                        r.worst_d = d;
                        r.worst_tag = tag;
                    }
                }
            }
        };
        if (mask != 0) {
            for (const auto& [tag, env] : m_tag_envelopes) {
                const auto it = m_tag_bit.find(tag);
                if (it != m_tag_bit.end() && (mask & (uint64_t(1) << it->second))) probe(env, tag);
            }
        } else if (r.offset_class) {
            probe(m_offset_envelope, -1);
        }
        bad.push_back(r);
    }

    if (bad.empty()) {
        logger().info(
            "\t[containment {}] clean: 0 of {} tracked faces outside ({} offset-class, {} "
            "region-class, {} neither)",
            when,
            n_tracked,
            n_offset_class,
            n_region_class,
            n_other);
        return;
    }

    logger().warn(
        "\t[containment {}] {} of {} tracked faces are OUTSIDE their envelope: {} OFFSET-class "
        "(the Phase A pin), {} REGION-class (a tag tube / junction intersection), {} neither "
        "| population: {} offset-class, {} region-class, {} neither",
        when,
        bad.size(),
        n_tracked,
        bad_offset,
        bad_region,
        bad_other,
        n_offset_class,
        n_region_class,
        n_other);

    std::sort(bad.begin(), bad.end(), [](const Bad& x, const Bad& y) {
        return x.worst_d > y.worst_d;
    });
    const size_t show = std::min<size_t>(bad.size(), 8);
    for (size_t i = 0; i < show; ++i) {
        const Bad& r = bad[i];
        const Vector3d& pa = m_vertex_attribute[r.v[0]].m_posf;
        logger().warn(
            "\t  [{}] face [{}, {}, {}] mask 0x{:x} longest edge {:.6g} | at ({:.6g}, {:.6g}, "
            "{:.6g}) | OUT BY {:.6g}; corners out by {:.6g}{}",
            r.offset_class ? "offset" : (r.mask ? "region" : "other "),
            r.v[0],
            r.v[1],
            r.v[2],
            r.mask,
            r.len,
            pa.x(),
            pa.y(),
            pa.z(),
            r.worst_d,
            r.worst_end_d,
            r.worst_tag >= 0 ? fmt::format(" (tag {})", r.worst_tag) : std::string());
        std::string corners;
        for (const size_t v : r.v) {
            const auto& ev = m_vertex_extra[v];
            corners += fmt::format(
                " v{}(mask 0x{:x} in/reg/off {}{}{})",
                v,
                ev.m_boundary_mask,
                int(ev.m_is_on_input),
                int(ev.m_is_on_region),
                int(ev.m_is_on_offset));
        }
        const auto found = try_tuple_from_face(r.v);
        logger().warn(
            "\t      corners:{} || face: surface_fs {} region {} offset {} label {} | live "
            "boundary "
            "bits 0x{:x}",
            corners,
            found ? m_face_attribute[std::get<1>(*found)].m_is_surface_fs : false,
            found ? face_is_region(std::get<1>(*found)) : false,
            found ? face_is_offset(std::get<1>(*found)) : false,
            found ? m_face_extra[std::get<1>(*found)].label : -1,
            found ? face_boundary_bits(std::get<0>(*found)) : uint64_t(0));
    }
}

void TopoOffsetTetMesh::log_region_face_mask_health(const std::string& when) const
{
    // Two counts, one invariant and one expectation -- see the 2D twin. The invariant is on the
    // stored masks: every tracked region face must dispatch to an envelope. The expectation is
    // that the LIVE bits go quiet once the band retags the cells it grew through.
    int n_region = 0, n_unmasked = 0, n_released = 0, n_live_dead = 0, n_wall = 0;
    int n_band = 0, n_outside = 0, n_mixed = 0, n_ends_offset = 0, n_ends_input = 0;
    size_t worst = size_t(-1);
    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        if (!face_is_region(fid)) continue;
        ++n_region;
        const std::optional<Tuple> opp = f.switch_tetrahedron(*this);
        if (!opp) ++n_wall;
        if (face_boundary_bits(f) == 0) ++n_live_dead;
        const std::array<size_t, 3> vs = get_face_vids(f);
        if (face_mask(vs) != 0) continue;
        if (face_boundary_bits(f) == 0) continue; // a quiet face bounds nothing any more
        if (!m_deform_tags.empty() && opp) {
            CellTag face_tags;
            const auto& t0 = m_tet_attribute[f.tid(*this)].tag;
            const auto& t1 = m_tet_attribute[opp->tid(*this)].tag;
            std::set_symmetric_difference(
                t0.begin(),
                t0.end(),
                t1.begin(),
                t1.end(),
                std::inserter(face_tags, face_tags.begin()));
            bool released_here = false;
            for (const int64_t t : face_tags) {
                if (m_deform_tags.count(t)) released_here = true;
            }
            if (released_here) {
                ++n_released;
                continue;
            }
        }
        ++n_unmasked;
        if (worst == size_t(-1)) worst = fid;
        if (n_unmasked <= 6) {
            std::string corners;
            for (const size_t v : vs) {
                const auto& A = m_vertex_attribute[v];
                const auto& EA = m_vertex_extra[v];
                corners += fmt::format(
                    " v{}(mask 0x{:x} in/reg/off {}{}{} bbox {}) at ({:.4g},{:.4g},{:.4g})",
                    v,
                    EA.m_boundary_mask,
                    int(EA.m_is_on_input),
                    int(EA.m_is_on_region),
                    int(EA.m_is_on_offset),
                    A.on_bbox_faces.size(),
                    A.m_posf.x(),
                    A.m_posf.y(),
                    A.m_posf.z());
            }
            logger().warn(
                "\t    unmasked f{}:{} | labels {} vs {}",
                fid,
                corners,
                m_tet_attribute[f.tid(*this)].label,
                opp ? std::to_string(m_tet_attribute[opp->tid(*this)].label) : std::string("-"));
        }
        const bool b0 = cell_is_offset_band(f.tid(*this));
        const bool b1 = opp && cell_is_offset_band(opp->tid(*this));
        if (b0 && b1) {
            ++n_band;
        } else if (!b0 && !b1) {
            ++n_outside;
        } else {
            ++n_mixed;
        }
        bool all_off = true, all_in = true;
        for (const size_t v : vs) {
            all_off = all_off && m_vertex_extra[v].m_is_on_offset;
            all_in = all_in && m_vertex_extra[v].m_is_on_input;
        }
        if (all_off) ++n_ends_offset;
        if (all_in) ++n_ends_input;
    }
    logger().info(
        "\t[envelope health @ {}] {} region-boundary faces tracked ({} on the wall) | {} freed "
        "by deform_others (released boundaries; expected) | {} with a ZERO stored mask (the "
        "invariant; must be 0) | {} with quiet LIVE bits (expected once the band retags the "
        "cells it grew through)",
        when,
        n_region,
        n_wall,
        n_released,
        n_unmasked,
        n_live_dead);

    {
        std::map<std::string, std::pair<int, int>> hist; // tag set -> (band cells, other cells)
        for (const Tuple& t : get_tets()) {
            const size_t tid = t.tid(*this);
            std::string key;
            for (const int64_t tg : m_tet_attribute[tid].tag) {
                key += (key.empty() ? "" : ",") + std::to_string(tg);
            }
            if (key.empty()) key = "-";
            auto& e = hist[key];
            (cell_is_offset_band(tid) ? e.first : e.second) += 1;
        }
        std::string tags;
        for (const auto& [k, v] : hist) {
            tags += fmt::format(
                "{}[{}] band {} / other {}",
                tags.empty() ? "" : " | ",
                k,
                v.first,
                v.second);
        }
        std::string bits;
        for (const auto& [t, b] : m_tag_bit) {
            bits += fmt::format("{}{}->bit{}", bits.empty() ? "" : " ", t, b);
        }
        logger()
            .info("\t[envelope health @ {}] cells by tag set: {} | tag bits: {}", when, tags, bits);
    }
    if (n_unmasked > 0) {
        logger().warn(
            "\t[envelope health @ {}] {} of {} tracked region-boundary faces ({:.1f}%) are "
            "contained by NOTHING (released boundaries already excluded): their corners' stored "
            "masks AND to zero, so surface_envelope_for_face() has no envelope to hold them to. "
            "Either a propagation hole, or collateral of deform_others' vertex freeing.",
            when,
            n_unmasked,
            n_region,
            100.0 * double(n_unmasked) / double(std::max(n_region, 1)));
        logger().warn(
            "\t[envelope health @ {}] of those {}: {} lie between two BAND cells, {} between two "
            "non-band cells, {} straddle the band surface | {} have every corner on the offset, "
            "{} every corner on the input complex | first is f{}",
            when,
            n_unmasked,
            n_band,
            n_outside,
            n_mixed,
            n_ends_offset,
            n_ends_input,
            worst);
    }
}

void TopoOffsetTetMesh::log_smooth_trace() const
{
    const auto& s = m_smooth_trace;
    logger().info(
        "\tsmooth trace: attempted {} | before: bbox {}, unrounded {}, phase-B wrong class {}, "
        "phase-B envelope-held background {} / ON-OFFSET {} | reached the smoother: {} on the "
        "offset surface, {} elsewhere ({} of them on another region boundary) | ({})",
        s.attempted.load(),
        s.before_bbox.load(),
        s.before_unrounded.load(),
        s.before_phase_b_not_offset.load(),
        s.before_phase_b_enveloped_background.load(),
        s.before_phase_b_enveloped_offset.load(),
        s.offset_attempted.load(),
        s.interior_attempted.load(),
        s.region_attempted.load(),
        m_smooth_rejects.to_string());
    if (s.before_phase_b_enveloped_offset.load() > 0) {
        logger().warn(
            "\t{} Phase B offset visits were SKIPPED AND PINNED: the vertex is on the offset "
            "surface AND held by an envelope (a tag region boundary, or the domain wall), which "
            "requires it to be within envelope_size of the input boundary and at target_distance "
            "from the complex at once. Not satisfiable, so it is left where Phase A put it and "
            "excluded from max_reachable rather than chased -- read the PINNED half of the band "
            "measures below for what that is costing.",
            s.before_phase_b_enveloped_offset.load());
    }
    if (m_placement_env_entry_outside.load() > 0) {
        logger().warn(
            "\t{} constrained placements found the vertex ALREADY outside its own envelope on "
            "entry. The invariant is 0 -- this says construction or Phase A is leaving offset "
            "vertices outside their region tube. The projection pulls them back in, so the "
            "placement still runs, but the violation upstream is real.",
            m_placement_env_entry_outside.load());
    }
    const int n = std::max(1, s.offset_attempted.load());
    logger().info(
        "\toffset term: {} attempted -> {} accepted | phi residual over them: avg {:.6} -> "
        "{:.6}, max {:.6} -> {:.6}",
        s.offset_attempted.load(),
        s.offset_accepted.load(),
        double(s.res_before_nano.load()) / n * 1e-9,
        double(s.res_after_nano.load()) / n * 1e-9,
        double(s.res_max_before_nano.load()) * 1e-9,
        double(s.res_max_after_nano.load()) * 1e-9);
}

// ---------------------------------------------------------------------------------------------
// The sizing field.
// ---------------------------------------------------------------------------------------------

void TopoOffsetTetMesh::init_offset_sizing_field()
{
    // Paper Sec. 5.3.3, Step 1: the sizing field "is defined on each edge of the offset mesh and
    // is initialized with the current length of each edge." The field is per-vertex here, so a
    // vertex takes the mean of its incident offset-surface edges; every other vertex is seeded
    // from its whole one-ring, or the first collapse pass decimates the background. As in 2D.
    const double l = std::max(m_params.l, 1e-16);
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / l);

    double raw_sum = 0.;
    int n_seeded = 0;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        double sum_len = 0.;
        int n = 0;
        std::set<size_t> seen;
        for (const Tuple& f : offset_surface_faces_live_at(vid)) {
            for (const size_t nb : get_face_vids(f)) {
                if (nb == vid || !seen.insert(nb).second) continue;
                sum_len += (m_vertex_attribute[vid].m_posf - m_vertex_attribute[nb].m_posf).norm();
                ++n;
            }
        }
        if (n == 0) {
            for (const size_t nb : get_one_ring_vids_for_vertex(vid)) {
                sum_len += (m_vertex_attribute[vid].m_posf - m_vertex_attribute[nb].m_posf).norm();
                ++n;
            }
            if (n == 0) continue; // isolated vertex; nothing to measure
            m_vertex_attribute[vid].m_sizing_scalar =
                std::clamp((sum_len / n) / l, s_floor, m_offset_params.max_sizing_scalar);
            continue;
        }
        raw_sum += sum_len / n;
        ++n_seeded;
        m_vertex_attribute[vid].m_sizing_scalar =
            std::clamp((sum_len / n) / l, s_floor, m_offset_params.max_sizing_scalar);
    }
    logger().info(
        "\tOffset sizing seed: {} vertices, mean incident length {:.6} -> target {:.6} "
        "(base l {:.6}, l_min {:.6} = {} x target_distance {}, scalar floor {:.6})",
        n_seeded,
        n_seeded > 0 ? raw_sum / n_seeded : 0.,
        std::max(n_seeded > 0 ? raw_sum / n_seeded : 0., m_offset_params.min_edge_length),
        l,
        m_offset_params.min_edge_length,
        m_offset_params.min_edge_length_rel,
        m_offset_params.target_distance,
        s_floor);

    // The front's resolution follows from the tolerance and the level set's curvature, set once
    // here: L <= sqrt(8 eps delta rho), rho the smaller principal radius of the level set of Phi
    // through the vertex, floored at delta. See the 2D twin.
    {
        const double delta = m_offset_params.target_distance;
        const double eps = m_offset_params.offset_envelope_rel;
        std::vector<size_t> changed;
        double L_min = std::numeric_limits<double>::infinity(), L_max = 0.;
        size_t n_flat = 0;
        for (const Tuple& v : get_vertices()) {
            const size_t vid = v.vid(*this);
            if (!m_vertex_extra[vid].m_is_on_offset) continue;
            const Vector3d x = m_vertex_attribute[vid].m_posf;
            const OffsetPotential3D& pot = potential_for(vid);
            const Vector3d g = pot.gradient(x);
            const Eigen::Matrix3d H = pot.hessian(x);
            const double gn = g.norm();
            // The largest principal curvature of the level set through x: the extreme eigenvalue
            // of the shape operator P H P / |g| on the tangent plane, P = I - n n^T.
            double rho = delta;
            if (gn > 0. && g.allFinite() && H.allFinite()) {
                const Vector3d n = g / gn;
                const Eigen::Matrix3d P = Eigen::Matrix3d::Identity() - n * n.transpose();
                const Eigen::Matrix3d S = P * H * P / gn;
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(S);
                double k = 0.;
                if (es.info() == Eigen::Success) {
                    for (int i = 0; i < 3; ++i) k = std::max(k, std::abs(es.eigenvalues()[i]));
                }
                if (std::isfinite(k) && k > 0.)
                    rho = std::max(1. / k, delta);
                else
                    rho = std::numeric_limits<double>::infinity();
            }
            if (!std::isfinite(rho)) {
                ++n_flat;
                continue; // a flat level set: the seeded resolution stands
            }
            const double L = 0.75 * std::sqrt(8. * eps * delta * rho);
            double& sc = m_vertex_attribute[vid].m_sizing_scalar;
            const double ns = std::clamp(L / l, s_floor, m_offset_params.max_sizing_scalar);
            if (ns < sc) {
                sc = ns;
                changed.push_back(vid);
            }
            const double Lc = std::min(L, sc * l);
            L_min = std::min(L_min, Lc);
            L_max = std::max(L_max, Lc);
        }
        if (!changed.empty()) gradation_smooth_sizing(m_offset_params.sizing_gradation, changed);
        logger().info(
            "\tFront resolution from the tolerance: L = 3/4 sqrt(8 eps delta rho), rho >= delta "
            "-> {:.6g} .. {:.6g} ({:.3g} .. {:.3g} x delta) at eps {}; {} front vertices "
            "tightened, {} on a flat level set left at the seed",
            std::isfinite(L_min) ? L_min : 0.,
            L_max,
            std::isfinite(L_min) ? L_min / delta : 0.,
            L_max / delta,
            eps,
            changed.size(),
            n_flat);
    }
}

// ---------------------------------------------------------------------------------------------
// The stall censuses and the needle diagnostics.
// ---------------------------------------------------------------------------------------------

void TopoOffsetTetMesh::log_refine_block_census(const std::string& when, const double filter_energy)
    const
{
    enum Verdict { kShort = 0, kValence, kContain, kFree, kNVerdict };
    static const char* kName[kNVerdict] = {"short", "valence", "contain", "free"};

    const double l = std::max(m_params.l, 1e-16);
    const size_t val_thresh = m_params.split_high_valence_threshold > 0
                                  ? size_t(m_params.split_high_valence_threshold)
                                  : std::numeric_limits<size_t>::max();
    static constexpr int E[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};

    std::array<size_t, kNVerdict> edge_hist{};
    std::array<size_t, kNVerdict> cell_first{};
    size_t n_bad = 0, n_inverted = 0, n_any_free = 0;
    size_t n_contain_offset = 0, n_contain_region = 0;

    struct Ex
    {
        double q = -1.;
        size_t tid = 0;
        Vector3d c = Vector3d::Zero();
        double dist = -1., phi_over_c = -1., sizing = 0.;
        std::array<double, 6> len{}, gate{};
        std::array<int, 6> verd{{-1, -1, -1, -1, -1, -1}};
        bool inverted = false;
        int label = -1;
    };
    std::array<Ex, kNVerdict> ex;

    const double c_level = m_offset_potential ? m_offset_potential->target_level() : 0.;

    for (size_t tid = 0; tid < tet_capacity(); ++tid) {
        if (!tuple_from_tet(tid).is_valid(*this)) continue;
        const double q = tet_amips(tid);
        if (!(q >= filter_energy)) continue;
        ++n_bad;

        const auto vs = oriented_tet_vids(tid);
        const bool inv = is_inverted(tuple_from_tet(tid));
        if (inv) ++n_inverted;

        Ex cand;
        cand.q = q;
        cand.tid = tid;
        cand.inverted = inv;
        cand.label = m_tet_attribute[tid].label;
        cand.sizing = std::numeric_limits<double>::max();
        for (const size_t v : vs) {
            cand.c += m_vertex_attribute[v].m_posf / 4.;
            cand.sizing = std::min(cand.sizing, m_vertex_attribute[v].m_sizing_scalar);
        }
        if (m_input_complex_bvh) {
            cand.dist = (cand.c - m_input_complex_bvh->nearest_point(cand.c)).norm();
        }
        if (m_offset_potential && c_level > 0.) {
            cand.phi_over_c = m_offset_potential->value(cand.c) / c_level;
        }

        int best = kShort;
        for (int k = 0; k < 6; ++k) {
            const size_t a = vs[size_t(E[k][0])], b = vs[size_t(E[k][1])];
            const Vector3d& pa = m_vertex_attribute[a].m_posf;
            const Vector3d& pb = m_vertex_attribute[b].m_posf;
            const double len2 = (pb - pa).squaredNorm();
            const double sr = 0.5 * (m_vertex_attribute[a].m_sizing_scalar +
                                     m_vertex_attribute[b].m_sizing_scalar);
            const double gate2 = m_params.splitting_l2 * sr * sr;
            cand.len[size_t(k)] = std::sqrt(len2);
            cand.gate[size_t(k)] = std::sqrt(std::max(gate2, 0.));

            Verdict v;
            if (len2 < gate2) {
                v = kShort;
            } else {
                // The link of the edge: the two other vertices of this tet stand in for it.
                bool valence_blocked = false;
                for (const size_t w : vs) {
                    if (w != a && w != b && vertex_valence(w) > val_thresh) valence_blocked = true;
                }
                if (valence_blocked) {
                    v = kValence;
                } else {
                    v = kFree;
                    // The child triangles' envelope is the parent's: the midpoint's mask is the
                    // endpoints' AND, so one dispatch serves both halves.
                    for (const size_t w : vs) {
                        if (w == a || w == b) continue;
                        const auto found = try_tuple_from_face({{a, b, w}});
                        if (!found) continue;
                        const size_t fid = std::get<1>(*found);
                        if (!m_face_attribute[fid].m_is_surface_fs) continue;
                        const std::shared_ptr<SampleEnvelope> env =
                            surface_envelope_for_face({{a, b, w}});
                        if (!env) continue;
                        const Vector3d mid = 0.5 * (pa + pb);
                        const Vector3d& pw = m_vertex_attribute[w].m_posf;
                        if (env->is_outside(std::array<Vector3d, 3>{{pa, mid, pw}}) ||
                            env->is_outside(std::array<Vector3d, 3>{{mid, pb, pw}})) {
                            v = kContain;
                            if (face_is_offset(fid))
                                ++n_contain_offset;
                            else
                                ++n_contain_region;
                            break;
                        }
                    }
                }
            }
            cand.verd[size_t(k)] = int(v);
            if (v == kFree)
                best = kFree;
            else if (best != kFree && v > best)
                best = v;
            ++edge_hist[size_t(v)];
        }
        ++cell_first[size_t(best)];
        if (best == kFree) ++n_any_free;
        if (cand.q > ex[size_t(best)].q) ex[size_t(best)] = cand;
    }

    if (n_bad == 0) {
        logger().info("\t[refine-block {}] no element at or above {:.4g}", when, filter_energy);
        return;
    }

    std::string cells, edges;
    for (int v = 0; v < kNVerdict; ++v) {
        if (cell_first[size_t(v)])
            cells +=
                fmt::format("{}{} {}", cells.empty() ? "" : ", ", cell_first[size_t(v)], kName[v]);
        if (edge_hist[size_t(v)])
            edges +=
                fmt::format("{}{} {}", edges.empty() ? "" : ", ", edge_hist[size_t(v)], kName[v]);
    }
    logger().info(
        "\t[refine-block {}] {} elements >= {:.4g} ({} exactly inverted) | best edge per element: "
        "{} | all {} edges: {} | containment refusals by class: {} offset, {} region | "
        "target l {:.6g}, split needs length >= {:.4g} x mean sizing",
        when,
        n_bad,
        filter_energy,
        n_inverted,
        cells,
        6 * n_bad,
        edges,
        n_contain_offset,
        n_contain_region,
        l,
        std::sqrt(std::max(m_params.splitting_l2, 0.)));

    logger().info(
        "\t[refine-block {}] {} of {} bad elements have at least one splittable edge -- for those "
        "the gates are NOT the obstacle",
        when,
        n_any_free,
        n_bad);

    for (int v = 0; v < kNVerdict; ++v) {
        const Ex& e = ex[size_t(v)];
        if (e.q < 0.) continue;
        std::string per_edge;
        for (int k = 0; k < 6; ++k) {
            per_edge += fmt::format(
                "{}{:.4g}/{:.4g} [{}]",
                k ? ", " : "",
                e.len[size_t(k)],
                e.gate[size_t(k)],
                kName[e.verd[size_t(k)]]);
        }
        logger().info(
            "\t  worst [{}]: t{} q {:.4g}{} label {} at ({:.6g}, {:.6g}, {:.6g}) | dist to complex "
            "{:.6g} = {:.4g}x delta | Phi/c {:.6g} | min sizing {:.6g} = {:.4g}x l | edges "
            "len/gate {}",
            kName[v],
            e.tid,
            e.q,
            e.inverted ? " INVERTED" : "",
            e.label,
            e.c.x(),
            e.c.y(),
            e.c.z(),
            e.dist,
            e.dist / std::max(m_offset_params.target_distance, 1e-16),
            e.phi_over_c,
            e.sizing,
            e.sizing / l,
            per_edge);
    }
}

void TopoOffsetTetMesh::log_stuck_refine_census(const double max_metric, const double filter_energy)
{
    ++m_stuck_calls;

    const double l = std::max(m_params.l, 1e-16);
    const double cell = l / 10.;

    size_t n_cells = 0, n_over_filter = 0, n_max = 0;
    size_t n_exact_inverted = 0, n_float_only = 0, n_unrounded = 0;
    std::array<size_t, 3> by_class{{0, 0, 0}}; // ambient / input complex / band
    size_t n_below_gate = 0, n_at_floor = 0;
    std::vector<double> volumes, shortest, aspects;
    std::vector<size_t> max_tids;
    std::set<std::tuple<long, long, long>> cells;
    static constexpr int E[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};

    for (size_t tid = 0; tid < tet_capacity(); ++tid) {
        const Tuple tt = tuple_from_tet(tid);
        if (!tt.is_valid(*this)) continue;
        ++n_cells;
        const double q = tet_amips(tid);
        if (q >= filter_energy) ++n_over_filter;
        if (cell_quality(tid) < MAX_ENERGY) continue;
        ++n_max;
        max_tids.push_back(tid);

        const auto vs = oriented_tet_vids(tid);
        const bool exact_bad = is_inverted(tt);
        const bool float_bad = is_inverted_f(tt);
        if (exact_bad)
            ++n_exact_inverted;
        else if (float_bad)
            ++n_float_only;
        bool any_unrounded = false;
        for (const size_t v : vs) any_unrounded |= !m_vertex_attribute[v].m_is_rounded;
        if (any_unrounded) ++n_unrounded;

        const int lab = m_tet_attribute[tid].label;
        by_class[lab >= 0 && lab <= 2 ? size_t(lab) : size_t(0)]++;

        std::array<Vector3d, 4> p;
        for (int i = 0; i < 4; ++i) p[size_t(i)] = m_vertex_attribute[vs[size_t(i)]].m_posf;
        volumes.push_back(std::abs((p[1] - p[0]).cross(p[2] - p[0]).dot(p[3] - p[0]) / 6.));
        double lo = std::numeric_limits<double>::max(), hi = 0.;
        for (const auto& e : E) {
            const double len = (p[size_t(e[0])] - p[size_t(e[1])]).norm();
            lo = std::min(lo, len);
            hi = std::max(hi, len);
        }
        shortest.push_back(lo);
        aspects.push_back(lo > 0. ? hi / lo : std::numeric_limits<double>::infinity());

        double sbar_hi = 0.;
        for (const size_t v : vs) sbar_hi += m_vertex_attribute[v].m_sizing_scalar;
        sbar_hi /= 4.;
        if (hi <= l * sbar_hi * 4. / 3.) ++n_below_gate;
        bool at_floor = true;
        for (const size_t v : vs)
            at_floor &= m_vertex_attribute[v].m_sizing_scalar <=
                        m_params.stuck_refine_min_scalar * (1. + 1e-9);
        if (at_floor) ++n_at_floor;

        const Vector3d ctr = (p[0] + p[1] + p[2] + p[3]) / 4.;
        cells.insert(
            {long(std::floor(ctr[0] / cell)),
             long(std::floor(ctr[1] / cell)),
             long(std::floor(ctr[2] / cell))});
    }

    if (n_max == 0) {
        logger().info(
            "[stuck-census #{}] {} tets, {} at or over filter {:.4}, NONE at MAX_ENERGY -- the "
            "stall is merely-bad elements, not degenerate ones (max metric {:.4})",
            m_stuck_calls,
            n_cells,
            n_over_filter,
            filter_energy,
            max_metric);
        m_stuck_prev_cells.clear();
        return;
    }

    auto pct = [&](size_t k) { return 100. * double(k) / double(n_max); };
    auto med = [](std::vector<double>& v) {
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    };

    // Connected clusters among the MAX_ENERGY tets, by shared face.
    std::unordered_map<size_t, size_t> idx_of;
    for (size_t i = 0; i < max_tids.size(); ++i) idx_of[max_tids[i]] = i;
    std::vector<size_t> parent(max_tids.size());
    std::iota(parent.begin(), parent.end(), size_t(0));
    std::function<size_t(size_t)> find = [&](size_t x) {
        while (parent[x] != x) x = parent[x] = parent[parent[x]];
        return x;
    };
    std::map<std::array<size_t, 3>, size_t> face_owner;
    for (size_t i = 0; i < max_tids.size(); ++i) {
        const auto vs = oriented_tet_vids(max_tids[i]);
        for (int skip = 0; skip < 4; ++skip) {
            std::array<size_t, 3> f = face_corners_from(vs, skip);
            std::sort(f.begin(), f.end());
            auto it = face_owner.find(f);
            if (it == face_owner.end()) {
                face_owner[f] = i;
            } else {
                const size_t ra = find(it->second), rb = find(i);
                if (ra != rb) parent[ra] = rb;
            }
        }
    }
    std::unordered_map<size_t, size_t> comp_size;
    for (size_t i = 0; i < max_tids.size(); ++i) comp_size[find(i)]++;
    size_t largest = 0;
    for (const auto& [root, sz] : comp_size) largest = std::max(largest, sz);

    size_t overlap = 0;
    for (const auto& c : cells)
        if (m_stuck_prev_cells.count(c)) ++overlap;
    const double overlap_pct =
        m_stuck_prev_cells.empty() ? 0. : 100. * double(overlap) / double(cells.size());

    logger().info(
        "[stuck-census #{}] {} tets | {} at/over filter {:.4} | {} at MAX_ENERGY ({:.2f}%)",
        m_stuck_calls,
        n_cells,
        n_over_filter,
        filter_energy,
        n_max,
        100. * double(n_max) / double(std::max<size_t>(n_cells, 1)));
    logger().info(
        "[stuck-census #{}]   cause: exactly inverted {} ({:.1f}%), float-degenerate only {} "
        "({:.1f}%), neither {} | with an unrounded vertex {} ({:.1f}%)",
        m_stuck_calls,
        n_exact_inverted,
        pct(n_exact_inverted),
        n_float_only,
        pct(n_float_only),
        n_max - n_exact_inverted - n_float_only,
        n_unrounded,
        pct(n_unrounded));
    logger().info(
        "[stuck-census #{}]   class: ambient {}, input complex {}, band {} | clusters {}, "
        "largest {} tets | grid cells {} ({:.1f}% shared with the previous census)",
        m_stuck_calls,
        by_class[0],
        by_class[1],
        by_class[2],
        comp_size.size(),
        largest,
        cells.size(),
        overlap_pct);
    logger().info(
        "[stuck-census #{}]   geometry: volume med {:.6g} (min {:.6g}), shortest edge med {:.6g}, "
        "aspect med {:.6g} | target l {:.6g}",
        m_stuck_calls,
        med(volumes),
        volumes.front(),
        med(shortest),
        med(aspects),
        l);
    logger().info(
        "[stuck-census #{}]   refinement applicable? {} of {} are ALREADY below the split gate "
        "({:.1f}%), {} are at the sizing floor {:.6g} ({:.1f}%)",
        m_stuck_calls,
        n_below_gate,
        n_max,
        pct(n_below_gate),
        n_at_floor,
        m_params.stuck_refine_min_scalar,
        pct(n_at_floor));

    const std::array<size_t, 6> now{
        {m_deg_split_created.load(),
         m_deg_collapse_offered.load(),
         m_deg_collapse_allowed.load(),
         m_deg_collapse_by_ringmax.load(),
         m_deg_collapse_by_stop.load(),
         m_deg_collapse_by_unrounded.load()}};
    logger().info(
        "[stuck-census #{}]   created since the last census: by SPLIT {} needle tets (a split "
        "is never refused on quality) | by COLLAPSE {} of {} offered at MAX_ENERGY, admitted by "
        "ring_max {} / stop_energy {} / unrounded {}",
        m_stuck_calls,
        now[0] - m_deg_prev_counts[0],
        now[2] - m_deg_prev_counts[2],
        now[1] - m_deg_prev_counts[1],
        now[3] - m_deg_prev_counts[3],
        now[4] - m_deg_prev_counts[4],
        now[5] - m_deg_prev_counts[5]);
    m_deg_prev_counts = now;

    m_stuck_prev_cells = std::move(cells);
}

bool TopoOffsetTetMesh::collapse_quality_allowed(
    const size_t v1,
    const double q,
    const double ring_max) const
{
    // Pure instrumentation: the base's rule -- TetWild's -- is returned unchanged.
    const bool allowed = TetOptimizerMesh::collapse_quality_allowed(v1, q, ring_max);
    if (q >= MAX_ENERGY) {
        ++m_deg_collapse_offered;
        if (allowed) {
            ++m_deg_collapse_allowed;
            if (!m_vertex_attribute.at(v1).m_is_rounded) {
                ++m_deg_collapse_by_unrounded;
            } else if (std::cbrt(q) <= m_params.stop_energy) {
                ++m_deg_collapse_by_stop;
            } else if (q <= ring_max) {
                ++m_deg_collapse_by_ringmax;
            }
        }
    }
    return allowed;
}

void TopoOffsetTetMesh::report_needle(const char* op, const size_t tid, const double parent_q) const
{
    if (m_needle_reports.fetch_add(1) >= kNeedleReports) return;

    const auto vs = oriented_tet_vids(tid);
    std::array<Vector3d, 4> p;
    for (int i = 0; i < 4; ++i) p[size_t(i)] = m_vertex_attribute[vs[size_t(i)]].m_posf;
    const double vol = (p[1] - p[0]).cross(p[2] - p[0]).dot(p[3] - p[0]) / 6.;
    static constexpr int E[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
    std::string edges;
    for (const auto& e : E) {
        edges += fmt::format(
            "{}{:.6g}",
            edges.empty() ? "" : " ",
            (p[size_t(e[0])] - p[size_t(e[1])]).norm());
    }

    std::string per_vertex;
    for (int k = 0; k < 4; ++k) {
        const size_t v = vs[size_t(k)];
        const auto& x = m_vertex_extra[v];
        per_vertex += fmt::format(
            "\n\t    v{} id {} ({:.17g}, {:.17g}, {:.17g}) input {} offset {} region {} mask "
            "0x{:x} "
            "epoch {} rounded {} sizing {:.6g}",
            k,
            v,
            p[size_t(k)][0],
            p[size_t(k)][1],
            p[size_t(k)][2],
            x.m_is_on_input,
            x.m_is_on_offset,
            x.m_is_on_region,
            x.m_boundary_mask,
            x.m_born_epoch,
            m_vertex_attribute[v].m_is_rounded,
            m_vertex_attribute[v].m_sizing_scalar);
    }
    logger().info(
        "[needle #{}] created at {} | tid {} label {} | volume {:.6g} | edges {} "
        "| parent AMIPS {} | is_inverted {} is_inverted_f {} | epoch {}{}",
        m_needle_reports.load(),
        op,
        tid,
        m_tet_attribute[tid].label,
        vol,
        edges,
        parent_q < 0. ? std::string("n/a") : fmt::format("{:.6g}", parent_q),
        is_inverted(tuple_from_tet(tid)),
        is_inverted_f(tuple_from_tet(tid)),
        m_op_epoch,
        per_vertex);
}

void TopoOffsetTetMesh::needle_scan(const char* when) const
{
    size_t n = 0;
    double worst_vol = std::numeric_limits<double>::max();
    size_t worst_tid = 0;
    std::array<size_t, 3> by_class{{0, 0, 0}};
    for (size_t tid = 0; tid < tet_capacity(); ++tid) {
        if (!tuple_from_tet(tid).is_valid(*this)) continue;
        if (tet_amips(tid) < kNeedleQuality) continue;
        ++n;
        const int lab = m_tet_attribute[tid].label;
        by_class[lab >= 0 && lab <= 2 ? size_t(lab) : size_t(0)]++;
        const auto vs = oriented_tet_vids(tid);
        const Vector3d& a = m_vertex_attribute[vs[0]].m_posf;
        const Vector3d& b = m_vertex_attribute[vs[1]].m_posf;
        const Vector3d& c = m_vertex_attribute[vs[2]].m_posf;
        const Vector3d& d = m_vertex_attribute[vs[3]].m_posf;
        const double vol = std::abs((b - a).cross(c - a).dot(d - a)) / 6.;
        if (vol < worst_vol) {
            worst_vol = vol;
            worst_tid = tid;
        }
    }
    if (n == 0) {
        logger().info("[needle-scan] {}: NONE", when);
        return;
    }
    logger().warn(
        "[needle-scan] {}: {} tets over AMIPS {:g} (ambient {}, input complex {}, band {}), "
        "smallest volume {:.6g} at tid {}",
        when,
        n,
        kNeedleQuality,
        by_class[0],
        by_class[1],
        by_class[2],
        worst_vol,
        worst_tid);
    needle_forensics();
    logger().info(
        "[needle-smooth] cumulative: {} visits with a needle in the ring | {} produced a "
        "candidate | {} actually repaired it | {} did not move the vertex at all",
        m_needle_smooth_offered.load(),
        m_needle_smooth_reached.load(),
        m_needle_smooth_fixed.load(),
        m_needle_smooth_stationary.load());
    report_needle("scan", worst_tid, -1.);
}

double TopoOffsetTetMesh::ring_max_quality(const size_t vid) const
{
    double m = -1.;
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        m = std::max(m, tet_amips(tid));
    }
    return m;
}

double TopoOffsetTetMesh::tet_flatness(const size_t tid) const
{
    const auto vs = oriented_tet_vids(tid);
    const Vector3d& a = m_vertex_attribute[vs[0]].m_posf;
    const Vector3d& b = m_vertex_attribute[vs[1]].m_posf;
    const Vector3d& c = m_vertex_attribute[vs[2]].m_posf;
    const Vector3d& d = m_vertex_attribute[vs[3]].m_posf;
    const double six_vol = std::abs((b - a).cross(c - a).dot(d - a));
    const double lmax = std::max(
        {(b - a).norm(),
         (c - a).norm(),
         (d - a).norm(),
         (c - b).norm(),
         (d - b).norm(),
         (d - c).norm()});
    return lmax > 0. ? six_vol / (lmax * lmax * lmax) : 0.;
}

void TopoOffsetTetMesh::record_flatness(
    const char* op,
    const double parent_flat,
    const size_t child_tid) const
{
    const double child = tet_flatness(child_tid);
    if (child >= kFlatThreshold) return;
    const bool from_healthy = parent_flat >= kFlatThreshold;
    if (from_healthy) {
        if (op[0] == 'S')
            ++m_flat_created_split;
        else
            ++m_flat_created_collapse;
    } else {
        ++m_flat_worsened_split;
    }
    if (from_healthy && m_flat_genesis_reports.fetch_add(1) < 10) {
        const auto vs = oriented_tet_vids(child_tid);
        std::string vtx;
        for (int k = 0; k < 4; ++k) {
            const auto& x = m_vertex_extra[vs[size_t(k)]];
            const Vector3d& p = m_vertex_attribute[vs[size_t(k)]].m_posf;
            vtx += fmt::format(
                "\n\t    v{} id {} ({:.17g}, {:.17g}, {:.17g}) epoch {} input {} region {} mask "
                "0x{:x}",
                k,
                vs[size_t(k)],
                p[0],
                p[1],
                p[2],
                x.m_born_epoch,
                x.m_is_on_input,
                x.m_is_on_region,
                x.m_boundary_mask);
        }
        logger().info(
            "[genesis #{}] {} turned a HEALTHY tet into a flat one: flatness {:.6g} -> {:.6g} "
            "(threshold {:g}) | tid {} label {} | AMIPS {:.6g}{}",
            m_flat_genesis_reports.load(),
            op,
            parent_flat,
            child,
            kFlatThreshold,
            child_tid,
            m_tet_attribute[child_tid].label,
            tet_amips(child_tid),
            vtx);
    }
}

void TopoOffsetTetMesh::needle_forensics() const
{
    const double l = std::max(m_params.l, 1e-16);
    const double coll_c = std::sqrt(std::max(m_params.collapsing_l2, 0.)); // = 4/5 l
    const double split_c = std::sqrt(std::max(m_params.splitting_l2, 0.)); // = 4/3 l
    static constexpr int E[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};

    std::vector<std::pair<double, size_t>> flat;
    for (size_t tid = 0; tid < tet_capacity(); ++tid) {
        if (!tuple_from_tet(tid).is_valid(*this)) continue;
        const double f = tet_flatness(tid);
        if (f < kFlatThreshold) flat.emplace_back(f, tid);
    }
    std::sort(flat.begin(), flat.end());
    if (flat.empty()) {
        logger().info("[forensics] 0 tets flatter than {:g}", kFlatThreshold);
    }
    if (!flat.empty())
        logger().warn(
            "[forensics] {} tets flatter than {:g} | gates: collapse 4/5*l = {:.6g}, split 4/3*l "
            "= {:.6g}, both scaled by the edge's mean sizing scalar",
            flat.size(),
            kFlatThreshold,
            coll_c,
            split_c);

    const size_t show = std::min<size_t>(flat.size(), 4);
    for (size_t i = 0; i < show; ++i) {
        const size_t tid = flat[i].second;
        const auto vs = oriented_tet_vids(tid);
        logger().warn(
            "[forensics] tet {} flatness {:.4g} AMIPS {:.6g} label {}",
            tid,
            flat[i].first,
            tet_amips(tid),
            m_tet_attribute[tid].label);
        for (const auto& e : E) {
            const size_t u = vs[size_t(e[0])], w = vs[size_t(e[1])];
            const double len = (m_vertex_attribute[u].m_posf - m_vertex_attribute[w].m_posf).norm();
            const double sbar =
                (m_vertex_attribute[u].m_sizing_scalar + m_vertex_attribute[w].m_sizing_scalar) /
                2.;
            const Tuple et = tuple_from_edge({{u, w}});
            const bool surf = const_cast<TopoOffsetTetMesh*>(this)->is_edge_on_surface(et);
            logger().warn(
                "[forensics]   edge {}-{} len {:.6g} | collapse gate {:.6g} -> {} | split gate "
                "{:.6g} -> {} | force-split queued {} | on_surface {}",
                u,
                w,
                len,
                coll_c * sbar,
                len <= coll_c * sbar ? "offered" : "NEVER OFFERED (too long)",
                split_c * sbar,
                len > split_c * sbar ? "SPLIT CANDIDATE" : "too short",
                is_force_split_edge(u, w),
                surf);
        }
    }

    // ---- coincident vertices ----
    const double eps = 1e-9 * l;
    std::unordered_map<long long, std::vector<size_t>> cells;
    const auto key = [&](const Vector3d& p) {
        return (long long)(std::llround(p[0] / (eps * 10.))) * 1000003LL * 1000003LL +
               (long long)(std::llround(p[1] / (eps * 10.))) * 1000003LL +
               (long long)(std::llround(p[2] / (eps * 10.)));
    };
    std::vector<size_t> live;
    for (const Tuple& v : get_vertices()) live.push_back(v.vid(*this));
    for (const size_t v : live) cells[key(m_vertex_attribute[v].m_posf)].push_back(v);
    size_t n_pairs = 0, n_pairs_no_edge = 0;
    std::string first;
    for (const auto& [k, group] : cells) {
        for (size_t i = 0; i < group.size(); ++i) {
            for (size_t j = i + 1; j < group.size(); ++j) {
                const double d =
                    (m_vertex_attribute[group[i]].m_posf - m_vertex_attribute[group[j]].m_posf)
                        .norm();
                if (d > eps) continue;
                ++n_pairs;
                const auto nbs = get_one_ring_vids_for_vertex(group[i]);
                const bool shares = std::find(nbs.begin(), nbs.end(), group[j]) != nbs.end();
                if (!shares) ++n_pairs_no_edge;
                if (first.empty()) {
                    first = fmt::format(
                        "first: {} and {} are {:.3g} apart, share an edge: {}",
                        group[i],
                        group[j],
                        d,
                        shares);
                }
            }
        }
    }
    if (n_pairs == 0) {
        logger().info("[forensics] coincident vertices (closer than {:.3g}): none", eps);
    }
    if (n_pairs > 0)
        logger().warn(
            "[forensics] coincident vertices (closer than {:.3g}): {} pairs, {} of them NOT joined "
            "by an edge (no collapse can reach those). {}",
            eps,
            n_pairs,
            n_pairs_no_edge,
            first.empty() ? "none" : first);
    logger().info(
        "[forensics] genesis tally: flat tets made from a HEALTHY parent -- split {}, collapse "
        "{} | flat-from-flat (multiplication) {}",
        m_flat_created_split.load(),
        m_flat_created_collapse.load(),
        m_flat_worsened_split.load());
}

// ---------------------------------------------------------------------------------------------
// The band's measures.
// ---------------------------------------------------------------------------------------------

std::vector<bool> TopoOffsetTetMesh::band_vertex_mask() const
{
    std::vector<bool> on_band(vert_capacity(), false);
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        for (const size_t vid : get_face_vids(f)) on_band[vid] = true;
    }
    return on_band;
}

double TopoOffsetTetMesh::band_vertex_distance_error(const size_t vid) const
{
    const Vector3d p = m_vertex_attribute[vid].m_posf;
    return std::abs(m_input_complex_bvh->dist(VectorXd(p)) - m_offset_params.target_distance);
}

double TopoOffsetTetMesh::band_vertex_residual(const size_t vid) const
{
    return potential_for(vid).residual_length(m_vertex_attribute[vid].m_posf);
}

TopoOffsetTetMesh::FaceSamples TopoOffsetTetMesh::offset_face_samples(const Tuple& f) const
{
    FaceSamples s;
    const int k = m_offset_params.offset_residual_samples;
    if (k <= 0) return s;
    for (const size_t v : get_face_vids(f)) {
        if (!band_vertex_is_reachable(v)) return s;
    }
    const OffsetPotential3D& pot = potential_for_face(f);
    for_each_offset_face_sample(f, [&](const Vector3d& q) {
        const double r = pot.residual_length(q);
        s.max = std::max(s.max, r);
        s.sum += r;
        ++s.n;
    });
    return s;
}

TopoOffsetTetMesh::DistanceSplit TopoOffsetTetMesh::residual_split() const
{
    // The band's Phi residual. Every offset-surface vertex and every face sample counts toward
    // the driving max, pinned ones included; the reachable/pinned split is attribution. Same as
    // 2D.
    const std::vector<bool> on_band = band_vertex_mask();

    DistanceSplit s;
    double sum_reachable = 0.;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!on_band[vid]) continue;
        const Vector3d p = m_vertex_attribute[vid].m_posf;
        const double err = potential_for(vid).residual_length(p);
        s.max_reachable = std::max(s.max_reachable, err);
        s.max_at_vertex = std::max(s.max_at_vertex, err);
        sum_reachable += err;
        ++s.n_reachable;
        if (band_vertex_is_reachable(vid)) {
            if (!potential_for(vid).within_support(p)) {
                ++s.n_outside_support;
                const double d = m_input_complex_bvh->dist(VectorXd(p));
                if (d > s.worst_outside_dist) {
                    s.worst_outside_dist = d;
                    s.worst_outside_vid = vid;
                }
            }
        } else {
            s.max_pinned = std::max(s.max_pinned, err);
            ++s.n_pinned;
        }
    }
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        const FaceSamples fs = offset_face_samples(f);
        if (fs.n == 0) continue;
        s.max_reachable = std::max(s.max_reachable, fs.max);
        s.max_in_face = std::max(s.max_in_face, fs.max);
        sum_reachable += fs.sum;
        s.n_reachable += fs.n;
    }

    s.avg_reachable = (s.n_reachable > 0) ? sum_reachable / s.n_reachable : 0.;
    return s;
}

TopoOffsetTetMesh::GradientSplit TopoOffsetTetMesh::gradient_split(
    const bool include_face_samples) const
{
    // ||grad (Phi(x) - c)^2|| at every band vertex, on the field the vertex is placed on, plus
    // the face-interior half on the same lattice the residual is sampled on. Same as 2D.
    const std::vector<bool> on_band = band_vertex_mask();
    std::vector<std::unique_ptr<OffsetEnergy3D>> energies;
    for (const auto& rp : m_region_potentials)
        energies.push_back(std::make_unique<OffsetEnergy3D>(rp, 1.0, true, true));
    OffsetEnergy3D union_energy(m_offset_potential, 1.0, true, true);
    const auto energy_for = [&](const int region) -> OffsetEnergy3D& {
        return (region >= 0 && size_t(region) < energies.size()) ? *energies[size_t(region)]
                                                                 : union_energy;
    };
    const auto project = [](const Eigen::VectorXd& g, const Vector3d& n) -> double {
        return (n.squaredNorm() > 0.) ? std::abs(g.head<3>().dot(n)) : g.norm();
    };

    GradientSplit s;
    double sum_reachable = 0.;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!on_band[vid]) continue;
        if (!m_vertex_extra[vid].m_is_on_offset) continue;

        if (!m_vertex_attribute[vid].m_is_rounded) {
            ++s.n_skipped_unrounded;
            continue;
        }
        const std::vector<Tuple> locs = get_one_ring_tets_for_vertex(v);
        if (locs.empty()) continue;
        bool inverted = false;
        for (const Tuple& loc : locs) {
            if (is_inverted_f(loc)) {
                inverted = true;
                break;
            }
        }
        if (inverted) {
            ++s.n_skipped_inverted;
            continue;
        }

        Eigen::VectorXd g(3);
        const Eigen::VectorXd x = m_vertex_attribute[vid].m_posf;
        energy_for(vertex_region(vid)).gradient(x, g);
        const double gn = g.norm();
        s.max_normal_aligned =
            std::max(s.max_normal_aligned, project(g, offset_vertex_normal(vid)));

        if (!band_vertex_is_reachable(vid)) {
            s.max_pinned = std::max(s.max_pinned, gn);
            ++s.n_pinned;
            continue;
        }

        if (gn > s.max_reachable) {
            s.max_reachable = gn;
            s.worst_vid = vid;
        }
        s.max_at_vertex = std::max(s.max_at_vertex, gn);
        sum_reachable += gn;
        ++s.n_reachable;
    }

    if (include_face_samples) {
        for (const Tuple& f : get_faces()) {
            if (!face_is_offset_surface_live(f)) continue;
            const auto vs = get_face_vids(f);
            const bool gating = band_vertex_is_reachable(vs[0]) &&
                                band_vertex_is_reachable(vs[1]) && band_vertex_is_reachable(vs[2]);
            const std::optional<Tuple> opp = f.switch_tetrahedron(*this);
            size_t band = f.tid(*this);
            if (!cell_is_offset_band(band) && opp) band = opp->tid(*this);
            const int region = band < m_cell_region.size() ? m_cell_region[band] : -1;
            for_each_offset_face_sample(f, [&](const Vector3d& q) {
                Eigen::VectorXd g(3);
                energy_for(region).gradient(Eigen::VectorXd(q), g);
                const double q_full = g.norm();
                if (gating) {
                    s.max_in_face = std::max(s.max_in_face, q_full);
                } else {
                    s.max_in_face_pinned = std::max(s.max_in_face_pinned, q_full);
                }
                ++s.n_face_samples;
            });
        }
    }

    s.avg_reachable = (s.n_reachable > 0) ? sum_reachable / s.n_reachable : 0.;
    return s;
}

double TopoOffsetTetMesh::edge_interpolation_residual(const size_t a, const size_t b) const
{
    const OffsetPotential3D& pot = potential_for_edge(a, b);
    const double c = pot.target_level();
    if (!(c > 0.)) return -1.;
    const auto r_at = [&](const Vector3d& p) { return (pot.value(p) - c) / c; };
    const Vector3d pa = m_vertex_attribute[a].m_posf, pb = m_vertex_attribute[b].m_posf;
    const double ra = r_at(pa), rb = r_at(pb), rm = r_at(0.5 * (pa + pb));
    if (!std::isfinite(ra) || !std::isfinite(rb) || !std::isfinite(rm)) return -1.;
    return 2. * (1. - m_params.w_amips) / c * std::abs(rm - 0.5 * (ra + rb));
}


TopoOffsetTetMesh::EnergyCriterion TopoOffsetTetMesh::energy_criterion()
{
    EnergyCriterion s;
    const OptPhase saved = m_phase;
    m_phase = OptPhase::B; // the objective's offset terms exist only in Phase B
    // The convergence length, front_conv_rel x delta: what "placed" means for a vertex and
    // "resolved" for a chord. See edge_conv_ratio().
    s.tube = m_offset_params.front_conv_rel * m_offset_params.target_distance;
    const auto placed = [&](const size_t vid) {
        return m_vertex_extra[vid].m_is_on_offset && m_vertex_attribute[vid].m_is_rounded;
    };
    std::vector<char> on_level(vert_capacity(), 0);
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!placed(vid)) continue;
        const Vector3d p = m_vertex_attribute[vid].m_posf;
        const double rho = potential_for(vid).residual_length(p);
        const double gn = front_vertex_conv_ratio(vid); // Newton step / (front_conv_rel x delta)
        const double newton_len =
            gn * m_offset_params.front_conv_rel * m_offset_params.target_distance;
        if (!std::isfinite(rho) || !std::isfinite(gn)) {
            ++s.n_unmeasurable;
        } else if (rho <= s.tube) {
            ++s.n_placed;
            on_level[vid] = 1;
        } else if (newton_len > s.tube) {
            ++s.n_travelling;
        } else if (front_vertex_touches_other(vid) || !m_offset_params.front_alignment_energy) {
            ++s.n_pressed_on;
            if (front_vertex_touches_other(vid)) ++s.n_pressed_touching;
        } else {
            ++s.n_stuck;
            if (rho > s.worst_stuck_rho) {
                s.worst_stuck_rho = rho;
                s.worst_stuck_vid = vid;
            }
        }
        if (vid < m_placement_pressed.size() && m_placement_pressed[vid]) {
            ++s.n_pressed; // constrained minimum: grad F is the constraint force, not a residual
            continue;
        }
        if (!std::isfinite(gn)) continue;
        ++s.n_vertices;
        if (gn > s.max_vertex) {
            s.max_vertex = gn;
            s.worst_vid = vid;
        }
    }
    for (const auto& e : offset_surface_edges()) {
        const size_t va = e[0], vb = e[1];
        if (!placed(va) || !placed(vb)) continue;
        if ((va < m_placement_pressed.size() && m_placement_pressed[va]) ||
            (vb < m_placement_pressed.size() && m_placement_pressed[vb])) {
            ++s.n_edges_pressed;
            continue;
        }
        const double gn = edge_conv_ratio(va, vb); // sag / tube
        if (gn < 0.) {
            ++s.n_unmeasurable;
            continue;
        }
        ++s.n_edges;
        if (gn > s.max_edge) {
            s.max_edge = gn;
            s.worst_edge_mid =
                0.5 * (m_vertex_attribute[va].m_posf + m_vertex_attribute[vb].m_posf);
            s.worst_edge_len =
                (m_vertex_attribute[va].m_posf - m_vertex_attribute[vb].m_posf).norm();
        }
        if (gn > s.bar) {
            ++s.n_edges_over;
            if (on_level[va] && on_level[vb]) {
                ++s.n_edges_over_on_level;
                const double len =
                    (m_vertex_attribute[va].m_posf - m_vertex_attribute[vb].m_posf).norm();
                // Refinable only if the rule can still lower a target; judged against the MAX of
                // the two scalars, as in 2D.
                const double l = std::max(m_params.l, 1e-300);
                const double s_floor = std::max(
                    m_offset_params.min_sizing_scalar,
                    m_offset_params.min_edge_length / l);
                const double target = front_chord_target(va, vb, len, gn * s.tube, s.tube);
                const double sn =
                    std::clamp(target / l, s_floor, m_offset_params.max_sizing_scalar);
                const double have = std::max(
                    m_vertex_attribute[va].m_sizing_scalar,
                    m_vertex_attribute[vb].m_sizing_scalar);
                if (sn < have) {
                    s.refinable.push_back({va, vb, gn * s.tube, len});
                } else {
                    ++s.n_at_floor;
                }
                if (gn > s.max_edge_on_level) {
                    s.max_edge_on_level = gn;
                    s.worst_on_level_mid =
                        0.5 * (m_vertex_attribute[va].m_posf + m_vertex_attribute[vb].m_posf);
                }
            }
        }
    }
    m_phase = saved;
    return s;
}

double TopoOffsetTetMesh::phase_b_front_gradient_linf()
{
    double worst = 0.;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!m_vertex_extra[vid].m_is_on_offset || !m_vertex_attribute[vid].m_is_rounded) continue;
        if (vid < m_placement_pressed.size() && m_placement_pressed[vid])
            continue; // constrained minimum
        const double gn = m_front_gradient_reference > 0. ||
                                  m_offset_params.front_conv_criterion != "gradient_norm_rel"
                              ? front_vertex_conv_ratio(vid)
                              : front_vertex_normal_gradient(vid);
        if (gn > worst) {
            worst = gn;
            m_front_gradient_worst_vid = vid;
        }
    }
    return worst;
}

double TopoOffsetTetMesh::front_vertex_conv_ratio(const size_t vid) const
{
    const double rel = m_offset_params.front_conv_rel;
    const std::string& crit = m_offset_params.front_conv_criterion;
    if (crit == "gradient_norm_rel") {
        const double bar = rel * m_front_gradient_reference;
        return bar > 0. ? front_vertex_normal_gradient(vid) / bar
                        : std::numeric_limits<double>::infinity();
    }
    const Vector3d x = m_vertex_attribute[vid].m_posf;
    Eigen::VectorXd xv = x, g(3);
    Eigen::MatrixXd H(3, 3);
    const auto prob = phase_b_front_objective(vid, x);
    prob->gradient(xv, g);
    prob->hessian(xv, H);
    if (!g.allFinite() || !H.allFinite()) return std::numeric_limits<double>::infinity();
    Vector3d n = front_vertex_move_direction(vid);
    if (!(n.squaredNorm() > 0.)) n = g.normalized();
    if (!(n.squaredNorm() > 0.)) return std::numeric_limits<double>::infinity();
    const double gn = n.dot(Vector3d(g)), h = n.dot(H * n);
    if (!(h > 0.)) return gn == 0. ? 0. : std::numeric_limits<double>::infinity();
    if (crit == "step_size_rel") {
        return std::abs(gn / h) / (rel * m_offset_params.target_distance);
    }
    const double F = prob->value(xv);
    return F > 0. ? (0.5 * gn * gn / h) / (rel * F) : std::numeric_limits<double>::infinity();
}

double TopoOffsetTetMesh::edge_conv_ratio(const size_t a, const size_t b) const
{
    const double r = edge_interpolation_residual(a, b);
    if (r < 0.) return r;
    const double rel = m_offset_params.front_conv_rel;
    if (m_offset_params.front_conv_criterion == "gradient_norm_rel") {
        const double bar = rel * m_front_gradient_reference;
        return bar > 0. ? r / bar : std::numeric_limits<double>::infinity();
    }
    // As a LENGTH: the sagitta of Phi over the chord, |Phi(m) - mean Phi|, divided by |grad Phi|
    // at the midpoint, against the accuracy front_conv_rel x target_distance. As in 2D.
    const OffsetPotential3D& pot = potential_for_edge(a, b);
    const Vector3d pa = m_vertex_attribute[a].m_posf, pb = m_vertex_attribute[b].m_posf;
    const Vector3d m = 0.5 * (pa + pb);
    const double gn = pot.gradient(m).norm();
    if (!(gn > 0.) || !std::isfinite(gn)) return -1.;
    const double sag = std::abs(pot.value(m) - 0.5 * (pot.value(pa) + pot.value(pb))) / gn;
    return sag / (m_offset_params.front_conv_rel * m_offset_params.target_distance);
}

void TopoOffsetTetMesh::assign_band_regions()
{
    // See m_region_potentials. A flood fill over the band cells, seeded from every band cell
    // with an input-complex vertex, whose piece is read off the per-piece BVHs (the nearest
    // piece to a vertex ON the complex is its own, at distance 0). As in 2D, a cell reached from
    // two regions and a vertex on cells of two regions read -2 and fall back to the union field.
    m_cell_region.assign(tet_capacity(), -1);
    m_vertex_region.assign(vert_capacity(), -1);
    if (m_n_regions <= 1 || m_region_potentials.empty() || m_region_bvhs.empty()) return;
    const auto region_at = [&](const Vector3d& p) -> int {
        int best = -1;
        double best_d = std::numeric_limits<double>::infinity();
        for (size_t r = 0; r < m_region_bvhs.size(); ++r) {
            const double d = m_region_bvhs[r]->squared_dist(VectorXd(p));
            if (d < best_d) {
                best_d = d;
                best = int(r);
            }
        }
        return best;
    };
    std::vector<size_t> queue;
    std::vector<int> piece_of_vertex(vert_capacity(), -3); // -3: not looked up yet
    for (const Tuple& t : get_tets()) {
        const size_t tid = t.tid(*this);
        if (!cell_is_offset_band(tid)) continue;
        for (const size_t v : oriented_tet_vids(tid)) {
            if (m_vertex_extra[v].label != 1) continue;
            if (piece_of_vertex[v] == -3) {
                piece_of_vertex[v] = region_at(m_vertex_attribute[v].m_posf);
            }
            const int r = piece_of_vertex[v];
            if (r < 0) continue;
            if (m_cell_region[tid] == -1) {
                m_cell_region[tid] = r;
                queue.push_back(tid);
            } else if (m_cell_region[tid] >= 0 && m_cell_region[tid] != r) {
                m_cell_region[tid] = -2;
            }
        }
    }
    while (!queue.empty()) {
        const size_t t = queue.back();
        queue.pop_back();
        const int r = m_cell_region[t];
        if (r < 0) continue;
        for (int j = 0; j < 4; ++j) {
            const std::optional<Tuple> opp = tuple_from_face(t, j).switch_tetrahedron(*this);
            if (!opp) continue;
            const size_t g = opp->tid(*this);
            if (!cell_is_offset_band(g)) continue;
            if (m_cell_region[g] == -1) {
                m_cell_region[g] = r;
                queue.push_back(g);
            } else if (m_cell_region[g] >= 0 && m_cell_region[g] != r) {
                m_cell_region[g] = -2;
            }
        }
    }
    std::vector<size_t> n_cells(size_t(m_n_regions), 0);
    size_t n_mixed_cells = 0, n_unreached = 0, n_mixed_verts = 0;
    for (size_t t = 0; t < m_cell_region.size(); ++t) {
        if (!tuple_from_tet(t).is_valid(*this) || !cell_is_offset_band(t)) continue;
        const int r = m_cell_region[t];
        if (r == -2) {
            ++n_mixed_cells;
            continue;
        }
        if (r < 0) {
            ++n_unreached;
            continue;
        }
        ++n_cells[size_t(r)];
        for (const size_t v : oriented_tet_vids(t)) {
            if (m_vertex_region[v] == -1) {
                m_vertex_region[v] = r;
            } else if (m_vertex_region[v] >= 0 && m_vertex_region[v] != r) {
                m_vertex_region[v] = -2;
                ++n_mixed_verts;
            }
        }
    }
    std::string per;
    for (size_t r = 0; r < n_cells.size(); ++r)
        per += fmt::format("{}{}", r ? " / " : "", n_cells[r]);
    if (n_mixed_cells > 0 || n_unreached > 0 || n_mixed_verts > 0) {
        logger().warn(
            "\t[regions] band cells per region {} | {} cells reached from TWO regions, {} reached "
            "from none, {} vertices on cells of two regions -- all fall back to the union field",
            per,
            n_mixed_cells,
            n_unreached,
            n_mixed_verts);
    } else {
        logger().info("\t[regions] band cells per region {}", per);
    }
}

void TopoOffsetTetMesh::log_front_profile(const size_t vid)
{
    if (vid == static_cast<size_t>(-1) || vid >= m_vertex_attribute.size() || !m_offset_potential)
        return;
    const int region = vertex_region(vid);
    const std::shared_ptr<const OffsetPotential3D> pot = potential_ptr_for(vid);
    const Vector3d x0 = m_vertex_attribute[vid].m_posf;
    Vector3d g = pot->gradient(x0);
    if (!(g.norm() > 0.) || !g.allFinite()) return;
    const Vector3d n = g / g.norm();
    const OptPhase saved = m_phase;
    m_phase = OptPhase::B;
    auto total = phase_b_front_energy(vid, pot);
    OffsetEnergy3D offset_only(pot, 1. - m_params.w_amips, true, true);
    const double delta = m_offset_params.target_distance;
    logger().info(
        "[front profile] worst vertex {} at ({:.5}, {:.5}, {:.5}), region {}, along the field "
        "direction n = ({:.4}, {:.4}, {:.4}); columns: s/delta | offset term | rest (alignment + "
        "w AMIPS) | total",
        vid,
        x0.x(),
        x0.y(),
        x0.z(),
        region,
        n.x(),
        n.y(),
        n.z());
    for (int k = -10; k <= 10; ++k) {
        const double sd = 0.05 * k;
        const Vector3d x = x0 + sd * delta * n;
        Eigen::VectorXd xv(3);
        xv << x.x(), x.y(), x.z();
        const double F = total->value(xv);
        const double Fo = offset_only.value(xv);
        logger().info("[front profile] {:+.2f} | {:.6g} | {:.6g} | {:.6g}", sd, Fo, F - Fo, F);
    }
    m_phase = saved;
}

bool TopoOffsetTetMesh::front_vertex_touches_other(const size_t vid) const
{
    // Through a BACKGROUND tet, one of whose other vertices is on the input, on a region
    // boundary, or on a front this vertex is not joined to by a front edge. A neighbour along
    // the same front does not count. As in 2D.
    std::set<size_t> along;
    for (const Tuple& f : offset_surface_faces_live_at(vid)) {
        for (const size_t u : get_face_vids(f)) {
            if (u != vid) along.insert(u);
        }
    }
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        if (m_tet_attribute[tid].label != 0) continue; // band or input: not the strip
        for (const size_t u : oriented_tet_vids(tid)) {
            if (u == vid) continue;
            const VertexExtra& ue = m_vertex_extra[u];
            if (ue.m_is_on_input || ue.m_is_on_region) return true;
            if (ue.m_is_on_offset && along.count(u) == 0) return true;
        }
    }
    return false;
}

double TopoOffsetTetMesh::front_chord_target(
    const size_t va,
    const size_t vb,
    const double len,
    const double sag,
    const double tube) const
{
    // 3/4 L (tube / sag)^(1/p), capped at L/2, with the exponent p measured from how the level
    // set turns across the chord: 2 on a smooth level set, 1 where the chord straddles a kink.
    // See the 2D twin for the derivation. The sag fraction a halving leaves is ratio = 2^-p, so
    // p = -log2(ratio): 1/4 -> 2, 1/2 -> 1. (The 2D twin writes -1 / log2(ratio), which maps
    // 1/4 to 0.5 and 1/2 to 1 and so over-refines every smooth chord by (sag / tube)^(1..2)
    // instead of the square root -- affordable on a curve, cubic on a surface. Not mirrored.)
    double p = 2.;
    const OffsetPotential3D& pot = potential_for_edge(va, vb);
    const Vector3d pa = m_vertex_attribute[va].m_posf, pb = m_vertex_attribute[vb].m_posf;
    const Vector3d ga = pot.gradient(pa), gb = pot.gradient(pb), gm = pot.gradient(0.5 * (pa + pb));
    const double na = ga.norm(), nb = gb.norm(), nm = gm.norm();
    if (std::isfinite(na) && na > 0. && std::isfinite(nb) && nb > 0. && std::isfinite(nm) &&
        nm > 0.) {
        const Vector3d ua = ga / na, ub = gb / nb, um = gm / nm;
        const auto turn = [](const Vector3d& u, const Vector3d& v) {
            return std::atan2(u.cross(v).norm(), u.dot(v));
        };
        const double phi = turn(ua, ub);
        if (phi > 0.) {
            const double ratio =
                std::clamp(0.5 * std::max(turn(ua, um), turn(um, ub)) / phi, 0.25, 0.5);
            p = -std::log2(ratio);
        }
    }
    return std::min(0.75 * len * std::pow(tube / sag, 1. / p), 0.5 * len);
}

size_t TopoOffsetTetMesh::refine_front_from_sag(
    const std::vector<EnergyCriterion::Refinable>& edges)
{
    const double l = std::max(m_params.l, 1e-300);
    const double tube = m_offset_params.front_conv_rel * m_offset_params.target_distance;
    const double s_floor =
        std::max(m_offset_params.min_sizing_scalar, m_offset_params.min_edge_length / l);
    std::vector<size_t> changed;
    for (const EnergyCriterion::Refinable& r : edges) {
        if (!(r.sag > 0.) || !(r.len > 0.)) continue;
        const double target = front_chord_target(r.a, r.b, r.len, r.sag, tube);
        const double sn = std::clamp(target / l, s_floor, m_offset_params.max_sizing_scalar);
        for (const size_t v : {r.a, r.b}) {
            double& sc = m_vertex_attribute[v].m_sizing_scalar;
            if (sn < sc) {
                sc = sn;
                changed.push_back(v);
            }
        }
    }
    if (!changed.empty()) gradation_smooth_sizing(m_offset_params.sizing_gradation, changed);
    return changed.size();
}

void TopoOffsetTetMesh::check_offset_within_support(const char* when) const
{
    report_outside_support(when, residual_split());
}

void TopoOffsetTetMesh::report_outside_support(const char* when, const DistanceSplit& s) const
{
    if (s.n_outside_support == 0) return;

    log_and_throw_error(
        "{}: {} offset-surface vertices have left the smooth offset potential's support "
        "(dhat = {} = offset_dhat_factor x target_distance {}). The worst is vertex {} at "
        "Euclidean distance {} from the input complex, which is {:.2f}x target_distance. Out "
        "there Phi is identically zero WITH a zero gradient: the smoothing term gives those "
        "vertices no direction back, their residual saturates instead of growing, and the "
        "sizing field refines around vertices nothing can move. Raise offset_dhat_factor if "
        "the offset legitimately has to travel that far, or reduce target_distance.",
        when,
        s.n_outside_support,
        m_offset_potential->dhat(),
        m_offset_params.target_distance,
        s.worst_outside_vid,
        s.worst_outside_dist,
        s.worst_outside_dist / std::max(m_offset_params.target_distance, 1e-16));
}

std::pair<double, double> TopoOffsetTetMesh::compute_distance_deviation() const
{
    const std::vector<bool> on_band = band_vertex_mask();

    double max_dist = 0.0;
    double sum_dist = 0.0;
    int n_verts = 0;
    m_worst_dist_vid = static_cast<size_t>(-1);
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!on_band[vid]) continue;
        ++n_verts;
        const double dist = band_vertex_distance_error(vid);
        if (dist > max_dist) {
            max_dist = dist;
            m_worst_dist_vid = vid;
        }
        sum_dist += dist;
    }
    const double avg_dist = (n_verts > 0) ? sum_dist / n_verts : 0.0;
    return std::make_pair(max_dist, avg_dist);
}

TopoOffsetTetMesh::DistanceSplit TopoOffsetTetMesh::distance_deviation_split() const
{
    const std::vector<bool> on_band = band_vertex_mask();

    DistanceSplit s;
    double sum_reachable = 0.;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!on_band[vid]) continue;
        const double err = band_vertex_distance_error(vid);
        if (band_vertex_is_reachable(vid)) {
            s.max_reachable = std::max(s.max_reachable, err);
            sum_reachable += err;
            ++s.n_reachable;
        } else {
            s.max_pinned = std::max(s.max_pinned, err);
            ++s.n_pinned;
        }
    }
    s.avg_reachable = (s.n_reachable > 0) ? sum_reachable / s.n_reachable : 0.;
    return s;
}

std::tuple<double, double> TopoOffsetTetMesh::optimization_quality_stats()
{
    // Phase A is TetWild, so its metric is TetWild's: element quality alone, in absolute AMIPS
    // against optimization_stop_metric() = stop_energy. Delegated rather than reimplemented, and
    // the units are the reason. Single runs TetWild's loop, so it takes the same.
    if (m_phase != OptPhase::B) {
        return wmtk::TetOptimizerMesh::optimization_quality_stats();
    }

    const DistanceSplit r = residual_split();
    report_outside_support("Optimization iteration", r);

    const double tol = offset_residual_tolerance();
    double amips = 0.;
    for (const Tuple& t : get_tets()) {
        amips = std::max(amips, cell_quality_rel(t.tid(*this)));
    }
    const double phi = r.max_reachable / tol;
    logger().info("\t[criteria] amips {:.4}x | phi {:.4}x", amips, phi);
    return {std::max(amips, phi), std::max(amips, r.avg_reachable / tol)};
}

double TopoOffsetTetMesh::cell_quality_rel(const size_t tid) const
{
    // cell_quality stores AMIPS^3 and stop_energy is a bar on AMIPS, so the cube root is not
    // optional.
    return std::cbrt(cell_quality(tid)) / std::max(m_params.stop_energy, 1e-16);
}

double TopoOffsetTetMesh::amips_rel_at_face(const Tuple& f) const
{
    double q = cell_quality_rel(f.tid(*this));
    if (const auto opp = f.switch_tetrahedron(*this)) {
        q = std::max(q, cell_quality_rel(opp->tid(*this)));
    }
    return q;
}

double TopoOffsetTetMesh::face_criterion_rel(const Tuple& f) const
{
    // The per-face form of optimization_quality_stats()'s Phase B max, restricted to what this
    // face carries. >= 1 means the face fails at least one criterion.
    const double tol = offset_residual_tolerance();
    double score = amips_rel_at_face(f);
    if (!face_is_offset_surface_live(f)) return score;
    for (const size_t vid : get_face_vids(f)) {
        if (!band_vertex_is_reachable(vid)) continue;
        score = std::max(score, band_vertex_residual(vid) / tol);
    }
    score = std::max(score, offset_face_samples(f).max / tol);
    return score;
}

size_t TopoOffsetTetMesh::refine_sizing_around_worst(const double max_metric)
{
    // TetWildMesh::refine_sizing_around_worst verbatim -- ranked by element quality, clamped the
    // same way, seeding the same force-split edges. Phase A only, by construction:
    // mesh_improvement() is this function's one caller, and the driver only ever runs that as
    // Phase A (the pre-optimisation pass and the frozen-front finishing pass).
    const int n_rings = std::max(0, m_params.stuck_refine_rings);
    const double filter_energy = std::min(std::max(max_metric / 100., m_params.stop_energy), 100.);

    // cell_quality is AMIPS^3, so the energy "max energy" refers to is its cube root.
    const auto worst = wmtk::utils::select_worst_cells(
        tet_capacity(),
        [this](size_t tid) { return tuple_from_tet(tid).is_valid(*this); },
        [this](size_t tid) { return std::cbrt(cell_quality(tid)); },
        filter_energy,
        m_params.stuck_refine_num_worst);
    if (worst.empty()) {
        return 0;
    }

    log_stuck_refine_census(max_metric, filter_energy);
    log_refine_block_census(fmt::format("stuck call {}", m_stuck_calls), filter_energy);

    m_force_split_edges.clear();
    if (m_params.stuck_refine_force_split) {
        for (const auto& [unused_score, tid] : worst) {
            m_force_split_edges.insert(
                wmtk::utils::longest_edge(
                    oriented_tet_vids(tid),
                    [this](size_t vid) -> const Vector3d& {
                        return m_vertex_attribute[vid].m_posf;
                    }));
        }
    }

    std::vector<size_t> seeds;
    seeds.reserve(4 * worst.size());
    for (const auto& [unused_score, tid] : worst) {
        for (const size_t v : oriented_tet_vids(tid)) seeds.push_back(v);
    }
    const auto region = wmtk::utils::grow_vertex_region(seeds, n_rings, [this](size_t v) {
        return get_one_ring_vids_for_vertex_adj(v);
    });

    const auto refined = wmtk::utils::apply_sizing_refinement(
        region,
        m_params.stuck_refine_factor,
        m_params.stuck_refine_min_scalar,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; });
    gradation_smooth_sizing(m_params.stuck_refine_gradation, refined);

    logger().info(
        "[stuck-refine A] worst {} tets (max energy {:.4}, filter {:.4}), refined {} of {} "
        "region vertices",
        worst.size(),
        max_metric,
        filter_energy,
        refined.size(),
        region.size());
    return refined.size();
}

void TopoOffsetTetMesh::log_worst_dist_vertex() const
{
    {
        const DistanceSplit r = residual_split();
        const DistanceSplit d = distance_deviation_split();
        logger().info(
            "\tband split (phi residual): {} reachable (max {:.6}, avg {:.6}) | {} PINNED "
            "(max {:.6})",
            r.n_reachable,
            r.max_reachable,
            r.avg_reachable,
            r.n_pinned,
            r.max_pinned);
        logger().info(
            "\tband split (euclidean dist err): {} reachable (max {:.6}, avg {:.6}) | {} PINNED "
            "(max {:.6})",
            d.n_reachable,
            d.max_reachable,
            d.avg_reachable,
            d.n_pinned,
            d.max_pinned);
    }

    const size_t vid = m_worst_dist_vid;
    if (vid == static_cast<size_t>(-1)) return;

    const Vector3d p = m_vertex_attribute[vid].m_posf;
    const double d = (p - m_input_complex_bvh->nearest_point(p)).norm();

    // Every face incident to vid, classified.
    const auto& ve = m_vertex_extra[vid];
    int n_offset_f = 0, n_region_f = 0, n_bbox_f = 0;
    std::set<size_t> seen_fids;
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        const auto tet_vids = oriented_tet_vids(tid);
        for (int skip = 0; skip < 4; ++skip) {
            if (tet_vids[size_t(skip)] == vid) continue;
            const auto [face_tuple, fid] = tuple_from_face(face_corners_from(tet_vids, skip));
            if (!seen_fids.insert(fid).second) continue;
            n_offset_f += face_is_offset_surface_live(face_tuple);
            n_region_f += face_is_region(fid);
            n_bbox_f += (m_face_attribute[fid].m_is_bbox_fs >= 0);
        }
    }
    logger().info(
        "\tworst-dist vertex {}: pos ({:.6}, {:.6}, {:.6}) dist {:.6} target {:.6} err {:.6}",
        vid,
        p[0],
        p[1],
        p[2],
        d,
        m_offset_params.target_distance,
        std::abs(d - m_offset_params.target_distance));
    logger().info(
        "\t  flags: on_offset {} on_input {} on_region {} on_bbox {} rounded {} | boundary mask "
        "{:#x} | incident faces: {} offset, {} region, {} bbox | phi {:.6} (level {:.6}), "
        "residual {:.6}, containment envelope {}",
        ve.m_is_on_offset,
        ve.m_is_on_input,
        ve.m_is_on_region,
        !m_vertex_attribute[vid].on_bbox_faces.empty(),
        m_vertex_attribute[vid].m_is_rounded,
        vertex_boundary_mask(vid),
        n_offset_f,
        n_region_f,
        n_bbox_f,
        potential_for(vid).value(p),
        potential_for(vid).target_level(),
        potential_for(vid).residual_length(p),
        smoothing_containment_envelope(vid) ? "yes" : "none");
    const char* fate =
        "Phase A: the shared smoother, AMIPS -- the offset term is NOT what moves it";
    if (!m_vertex_attribute[vid].m_is_rounded) {
        fate = "REFUSED by smooth_before: not rounded";
    } else if (phase_places_front() && ve.m_is_on_offset) {
        fate = "Phase B / single: the local root find on (Phi - c)^2";
    } else if (phase_places_front()) {
        fate = "Phase B: interior AMIPS, or refused if it carries a surface";
    }
    logger().info("\t  smoothing fate: {}", fate);

    const auto tags_to_string = [](const CellTag& tags) {
        std::string s = "{";
        for (const int64_t t : tags) {
            if (s.size() > 1) s += ",";
            s += std::to_string(t);
        }
        return s + "}";
    };
    seen_fids.clear();
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        const auto tet_vids = oriented_tet_vids(tid);
        for (int skip = 0; skip < 4; ++skip) {
            if (tet_vids[size_t(skip)] == vid) continue;
            const auto [ft, fid] = tuple_from_face(face_corners_from(tet_vids, skip));
            if (!seen_fids.insert(fid).second) continue;
            const char* cls = "untracked";
            if (m_face_attribute[fid].m_is_surface_fs) {
                cls = m_face_attribute[fid].m_surface_class == OFFSET_SURFACE_CLASS ? "OFFSET"
                                                                                    : "REGION";
            } else if (m_face_attribute[fid].m_is_bbox_fs >= 0) {
                cls = "bbox";
            }
            const auto fv = get_face_vids(ft);
            const std::optional<Tuple> opp = ft.switch_tetrahedron(*this);
            if (!opp) {
                logger().info(
                    "\t  face [{}, {}, {}]: class {} (domain boundary, one tet)",
                    fv[0],
                    fv[1],
                    fv[2],
                    cls);
                continue;
            }
            const size_t ta = ft.tid(*this), tb = opp->tid(*this);
            logger().info(
                "\t  face [{}, {}, {}]: class {} | tets {} tags {} label {} band {} | {} tags {} "
                "label {} band {}",
                fv[0],
                fv[1],
                fv[2],
                cls,
                ta,
                tags_to_string(m_tet_attribute[ta].tag),
                m_tet_attribute[ta].label,
                cell_is_offset_band(ta),
                tb,
                tags_to_string(m_tet_attribute[tb].tag),
                m_tet_attribute[tb].label,
                cell_is_offset_band(tb));
        }
    }
}

void TopoOffsetTetMesh::check_no_vertex_on_both_surfaces(const char* when) const
{
    // A vertex on both surfaces is unsatisfiable: at distance 0 from the input complex and
    // required to sit at target_distance from it. The geometry decides, not the flags, which are
    // over-broad. Checked after every phase, not only at construction. As in 2D.
    std::vector<size_t> both;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!m_vertex_extra[vid].m_is_on_offset || !m_vertex_extra[vid].m_is_on_input) {
            continue;
        }
        if (m_input_complex_bvh->dist(VectorXd(m_vertex_attribute[vid].m_posf)) >
            m_offset_params.envelope_size) {
            continue;
        }
        both.push_back(vid);
    }
    if (both.empty()) {
        return;
    }

    const size_t n_show = std::min<size_t>(both.size(), 8);
    std::string detail;
    for (size_t i = 0; i < n_show; ++i) {
        const size_t vid = both[i];
        const double d = m_input_complex_bvh->dist(VectorXd(m_vertex_attribute[vid].m_posf));
        detail += fmt::format("{}{} (dist to input {:.6g})", i ? ", " : "", vid, d);
    }
    log_and_throw_error(
        "[{}] {} vertices are on BOTH the input complex and the offset surface. Such a vertex is "
        "at distance 0 from the input and is asked to be at target_distance {} from it at the "
        "same time, so the optimization cannot place it and the offset surface through it cannot "
        "converge. This is a construction defect, not an optimization failure. Offending "
        "vertices: {}{}",
        when,
        both.size(),
        m_offset_params.target_distance,
        detail,
        both.size() > n_show ? ", ..." : "");
}

void TopoOffsetTetMesh::rebuild_offset_envelope()
{
    // The released boundaries' ops-only tube: mark and rebuild NOW, at this consistent moment.
    m_released_tube_dirty.store(true, std::memory_order_release);
    released_envelope();
    // First, and on every path out of here including the empty one: each entry is an
    // IntersectionEnvelope holding the tube this call is about to replace.
    {
        std::lock_guard<std::mutex> lock(m_isect_mutex);
        m_offset_isect_cache.clear();
    }

    std::vector<Eigen::Vector3i> tris;
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        const auto vs = get_face_vids(f);
        tris.emplace_back(int(vs[0]), int(vs[1]), int(vs[2]));
    }
    if (tris.empty()) {
        m_offset_envelope = nullptr;
        logger().warn("\t[offset envelope] no offset-surface faces; the envelope is empty");
        return;
    }

    std::vector<Eigen::Vector3d> verts(vert_capacity());
    for (size_t i = 0; i < vert_capacity(); ++i) {
        verts[i] = m_vertex_attribute[i].m_posf;
    }

    // A straight fraction of target_distance and nothing else: both are distances in model
    // units, so offset_envelope_rel is a pure percentage. As in 2D.
    const double eps =
        std::max(m_offset_params.offset_envelope_rel * m_offset_params.target_distance, 1e-12);

    m_offset_envelope = std::make_shared<SampleEnvelope>(/*exact=*/true);
    m_offset_envelope->init(verts, tris, eps);
    logger().info(
        "\t[offset envelope] rebuilt: {} faces, {} (eps {:.6g} = "
        "offset_envelope_rel {:.4} x target_distance {:.6g})",
        tris.size(),
        m_offset_envelope->use_exact ? "EXACT" : "sampled",
        eps,
        m_offset_params.offset_envelope_rel,
        m_offset_params.target_distance);
}

void TopoOffsetTetMesh::append_frame_label(const size_t idx, const std::string& label) const
{
    std::ofstream f(
        m_offset_params.output_path + "_frames.txt",
        idx == 0 ? std::ios::trunc : std::ios::app);
    if (f) f << fmt::format("{:05d}\t{}\n", idx, label);
}

// ---------------------------------------------------------------------------------------------
// The loop.
// ---------------------------------------------------------------------------------------------

void TopoOffsetTetMesh::optimize_offset_single_phase()
{
    // One phase. Phase A is already TetWild's mesh_improvement -- split / smooth / collapse /
    // smooth / swap / smooth -- so operations and smoothing interleave there already; the only
    // things Phase B adds are WHICH objective a front vertex is smoothed against and that it is
    // not caged in the offset tube while it moves. Give A's smoothing passes both
    // (OptPhase::Single) and the second phase has nothing left to do. The tube still holds the
    // front for the OPERATIONS (surface_envelope_for_face, which does not read the phase) and is
    // rebuilt after every group, so it follows the front rather than capping it. As in 2D.
    const int rounds = std::max(1, m_offset_params.max_rounds);
    const int a_iters = std::max(1, m_offset_params.max_iterations);
    check_no_vertex_on_both_surfaces("construction");
    log_region_face_mask_health("construction");
    audit_surface_containment("construction");
    needle_scan("after construction, before the single-phase loop");
    assign_band_regions();
    m_phase = OptPhase::B; // the reference is measured with the offset terms present
    m_front_gradient_reference = phase_b_front_gradient_linf();
    logger().info(
        "\tSINGLE PHASE: TetWild's loop with the front placed inside its "
        "smoothing passes | front energy-gradient reference {:.6g}, criterion {} at rel {}",
        m_front_gradient_reference,
        m_offset_params.front_conv_criterion,
        m_offset_params.front_conv_rel);
    (void)rounds;
    const int budget = std::max(1, m_offset_params.max_rounds);
    // One turn is TetWild's operation groups, run here rather than through mesh_improvement() so
    // the tube can be rebuilt AFTER EVERY SMOOTHING PASS. What mesh_improvement() adds and is
    // left out here on purpose is its stall response, which refines around the worst elements: a
    // travelling front stretches cells by design.
    const int k = std::max(1, m_params.interleaved_smoothing_passes);
    const std::array<std::array<int, 4>, 3> groups = {
        {{{1, 0, 0, k}}, {{0, 1, 0, k}}, {{0, 0, 1, k}}}}; // split | collapse | swap, each + smooth
    static constexpr std::array<const char*, 3> group_names = {{"split", "collapse", "swap"}};
    compute_vertex_partition_morton();
    // One turn of grace after a target is lowered: refine_front_from_sag() writes a sizing target
    // and the split pass that realizes it does not run until the NEXT turn.
    size_t lowered_last_turn = 0;
    for (int it = 0; it < budget; ++it) {
        m_ab_round = it + 1;
        m_iterations_used = it + 1;
        m_phase = OptPhase::Single;
        for (const Tuple& v : get_vertices()) {
            const size_t vid = v.vid(*this);
            m_vertex_extra[vid].m_turn_start = m_vertex_attribute[vid].m_posf;
            m_vertex_extra[vid].m_turn_start_valid = true;
        }
        rebuild_offset_envelope();
        for (size_t gi = 0; gi < groups.size(); ++gi) {
            stamp_plastic_rests(); // plastic: each group resists only its own increment
            if (gi == 1) needle_scan("collapse pass");
            m_debug_pass_name = group_names[gi];
            local_operations(groups[gi]);
            rebuild_offset_envelope(); // the smoothing in this group moved the front
        }
        consolidate_mesh();
        assign_band_regions();
        const double amips = std::get<0>(optimization_quality_stats());
        const double bar = optimization_stop_metric();
        const EnergyCriterion ec = energy_criterion();
        const Vector3d wx = ec.worst_vid != static_cast<size_t>(-1)
                                ? m_vertex_attribute[ec.worst_vid].m_posf
                                : Vector3d::Zero();
        logger().info(
            "======== single-phase turn {} / {}: max AMIPS {:.4} (stop {:.4}) | front vertices "
            "max {:.4}x the bar (worst v{} at ({:.4}, {:.4}, {:.4})), edges max {:.4}x (reported) "
            "| {} vertices, {} edges | edges over the tube: {}, of which {} with both ends on the "
            "level set (worst {:.4}x at ({:.4}, {:.4}, {:.4})) | states: placed {} pressed {} "
            "travelling {} stuck {} | refinable edges {} (at the sizing floor {}) ========",
            it + 1,
            budget,
            amips,
            bar,
            ec.max_vertex,
            ec.worst_vid,
            wx.x(),
            wx.y(),
            wx.z(),
            ec.max_edge,
            ec.n_vertices,
            ec.n_edges,
            ec.n_edges_over,
            ec.n_edges_over_on_level,
            ec.max_edge_on_level,
            ec.worst_on_level_mid.x(),
            ec.worst_on_level_mid.y(),
            ec.worst_on_level_mid.z(),
            ec.n_placed,
            ec.n_pressed_on,
            ec.n_travelling,
            ec.n_stuck,
            ec.refinable.size(),
            ec.n_at_floor);
        if (m_offset_params.debug_output) {
            write_optimization_debug_output(fmt::format("phase_{}S", it + 1));
        }
        const size_t lowered_prev = lowered_last_turn;
        lowered_last_turn = 0;
        if (!ec.refinable.empty()) {
            const size_t n = refine_front_from_sag(ec.refinable);
            lowered_last_turn = n;
            logger().info(
                "\t[resolution] turn {}: {} front edge(s) with both ends on the level set sag over "
                "the tube (worst {:.4}x at ({:.4}, {:.4}, {:.4})) -> target lowered at {} vertices",
                it + 1,
                ec.refinable.size(),
                ec.max_edge_on_level,
                ec.worst_on_level_mid.x(),
                ec.worst_on_level_mid.y(),
                ec.worst_on_level_mid.z(),
                n);
        }
        if (ec.n_stuck > 0) {
            const Vector3d sp = m_vertex_attribute[ec.worst_stuck_vid].m_posf;
            logger().info(
                "\t[stuck] turn {}: {} front vertex(es) stopped short of the level set touching "
                "nothing; worst v{} at ({:.4}, {:.4}, {:.4}), {:.4} x delta off",
                it + 1,
                ec.n_stuck,
                ec.worst_stuck_vid,
                sp.x(),
                sp.y(),
                sp.z(),
                ec.worst_stuck_rho / m_offset_params.target_distance);
        }
        // Termination: every front vertex placed or pressed, none travelling or stuck, and no
        // chord left to resolve -- then quality with the front frozen (below).
        if (ec.converged_single() && lowered_prev == 0) {
            m_energy_verdict = ec;
            m_converged = true;
            logger().info(
                "Single phase: the front is placed after {} iteration(s) (phi {:.4}x); max AMIPS "
                "{:.4} against stop {:.4}",
                it + 1,
                ec.max_vertex / ec.bar,
                amips,
                bar);
            if (amips >= bar) {
                logger().info(
                    "======== final pass, front frozen: max AMIPS {:.6g} >= stop_energy {} "
                    "========",
                    amips,
                    m_params.stop_energy);
                m_ab_round = it + 2;
                m_phase = OptPhase::A;
                rebuild_offset_envelope();
                m_freeze_front = true;
                mesh_improvement(a_iters);
                m_freeze_front = false;
                assign_band_regions();
                const double final_amips = std::get<0>(optimization_quality_stats());
                logger().info(
                    "\t[final pass] max element quality {:.4} (stop {:.4}) -> {}",
                    final_amips,
                    optimization_stop_metric(),
                    final_amips < m_params.stop_energy ? "ok" : "STILL OVER");
                if (m_offset_params.debug_output) {
                    write_optimization_debug_output(fmt::format("phase_{}A", it + 2));
                }
            }
            rebuild_offset_envelope();
            return;
        }
    }
    logger().warn("Single phase did not converge in {} turns (max_rounds)", budget);
    log_front_profile(energy_criterion().worst_vid);
}

void TopoOffsetTetMesh::optimize_offset(const std::filesystem::path& output_file)
{
    logger().info("Optimizing offset (3D)...");

    // From here on every edge split is an optimization split, run by the shared engine.
    m_edge_split_mode = EdgeSplitMode::Optimization;

    // label the offset surface, and with it the vertices the optimization places
    logger().info("\tLabel offset faces...");
    label_offset_boundary();

    init_vertex_order();

    // deform_others: from here on, other input regions deform instead of being envelope-held.
    if (m_offset_params.deform_others) {
        release_deformable_regions();
        if (!m_deform_tags.empty()) {
            m_plastic_active = true;
            stamp_plastic_rests();
        }
    }

    // The offset envelope is born here, with the offset itself, and always exists from then on.
    rebuild_offset_envelope();

    // The front as constructed must already be inside the potential's support.
    check_offset_within_support("Offset as constructed");

    logger().info(
        "\tOffset criterion: |grad (Phi - c)^2 . n| <= front_conv_rel {} x "
        "max|grad (Phi - c)^2 . n| over the band AS CONSTRUCTED, with n the unit normal from "
        "the offset surface's own normal (Voronoi-weighted at vertices, the face's own inside "
        "a face). Measured over every band vertex and {} sample(s) "
        "per band face; the reference is reported next, before the loop starts.",
        m_offset_params.front_conv_rel,
        offset_residual_samples());

    // Seed the sizing field from the offset's current edge lengths, before any operation runs.
    init_offset_sizing_field();

    // Unconditional: write_vtu() must not be the only consolidate here (see the 2D twin).
    consolidate_mesh();
    if (m_offset_params.debug_output) {
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

    iter_cnt_split = 0;
    iter_cnt_split_born = 0;
    iter_cnt_recollapsed = 0;
    iter_cnt_recollapsed_same_pass = 0;
    iter_cnt_collapse = 0;
    iter_cnt_collapse_offset_removed = 0;
    iter_cnt_swap = 0;
    m_smooth_trace.reset();
    optimization_metrics.clear();
    op_counts.clear();
    churn_counts.clear();

    // Frame 0 is the mesh as constructed, before the optimization touches it.
    if (m_params.debug_output) {
        m_debug_pass_name = "construction";
        write_optimization_debug_output(fmt::format("debug_{}", m_debug_print_counter++));
    }

    optimize_offset_single_phase();

    log_smooth_trace();
    logger().info(
        "splits = {} (offset-edge: {} offered -> {} accepted)  |  collapses = {} ({} removed an "
        "offset vertex, {} refused by the offset criterion)  |  swaps = {} ({} refused by the "
        "offset criterion)",
        iter_cnt_split.load(),
        iter_cnt_split_offset_before.load(),
        iter_cnt_split_offset.load(),
        iter_cnt_collapse.load(),
        iter_cnt_collapse_offset_removed.load(),
        iter_cnt_collapse_offset_reject.load(),
        iter_cnt_swap.load(),
        iter_cnt_swap_offset_reject.load());

    // Final metrics and the convergence verdict, one entry for the whole run.
    assign_band_regions();
    const auto [max_dist, avg_dist] = compute_distance_deviation();
    const DistanceSplit r = residual_split();
    const GradientSplit g = gradient_split();
    const double tol = offset_residual_tolerance();
    const double gtol = offset_gradient_tolerance();
    logger().info(
        "placement gradient (at band vertices): max {} (avg {}) vs tolerance {} "
        "[front_conv_rel {}] | in-face diagnostic {} ({} face samples) | {} "
        "reachable, {} pinned (max {}), {} skipped ({} unrounded, {} inverted ring)",
        g.max_reachable,
        g.avg_reachable,
        gtol,
        m_offset_params.front_conv_rel,
        g.max_in_face,
        g.n_face_samples,
        g.n_reachable,
        g.n_pinned,
        g.max_pinned,
        g.n_skipped_unrounded + g.n_skipped_inverted,
        g.n_skipped_unrounded,
        g.n_skipped_inverted);
    logger().info(
        "phi residual (diagnostic, absolute model units): max {} (avg {}) vs bar {} | at "
        "vertices {}, inside faces {} | {} samples, {} pinned vertices || euclid dist err: max {} "
        "| avg {}",
        r.max_reachable,
        r.avg_reachable,
        tol,
        r.max_at_vertex,
        r.max_in_face,
        r.n_reachable,
        r.n_pinned,
        max_dist,
        avg_dist);
    optimization_metrics.push_back(
        {{max_dist,
          avg_dist,
          r.max_reachable,
          r.avg_reachable,
          g.max_reachable,
          g.avg_reachable,
          g.max_at_vertex,
          g.max_in_face}});
    log_worst_dist_vertex();

    {
        const EnergyCriterion ec = m_energy_verdict ? *m_energy_verdict : energy_criterion();
        m_converged = ec.converged_single();
        logger().log(
            m_converged ? spdlog::level::info : spdlog::level::warn,
            "{}{}: front vertices placed {} / pressed {} / travelling {} / stuck {} | "
            "chords to resolve {} (at the sizing floor {}) | accuracy front_conv_rel {} "
            "x target_distance = {:.4} | (vertex test, informative: max {:.4}x its bar)",
            m_converged ? "Converged" : "Optimization did not converge",
            m_energy_verdict ? " (measured at convergence, before the finishing pass)" : "",
            ec.n_placed,
            ec.n_pressed_on,
            ec.n_travelling,
            ec.n_stuck,
            ec.refinable.size(),
            ec.n_at_floor,
            m_offset_params.front_conv_rel,
            ec.tube,
            ec.max_vertex);
    }

    if (!m_converged && m_offset_params.throw_on_nonconvergence) {
        log_and_throw_error(
            "Optimization did not converge and throw_on_nonconvergence is set. Ran {} of {} "
            "iterations; see the warnings above for the criterion that failed.",
            optimization_metrics.size(),
            m_offset_params.max_iterations);
    }
}

} // namespace wmtk::components::topological_offset
