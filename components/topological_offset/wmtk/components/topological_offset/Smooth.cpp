
#include "TopoOffsetTetMesh.h"

#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>

#include <cmath>
#include <set>
#include <vector>

namespace wmtk::components::topological_offset {

/**
 * The smoothing hooks: which objective a vertex gets, and what is refused before the smoother
 * sees it. The front's own placement is FrontSmooth3d.cpp; everything here mirrors the 2D
 * smooth_before() / smooth_after() in Optimize2d.cpp.
 */

namespace {
void atomic_max(std::atomic<long long>& target, long long value)
{
    long long cur = target.load();
    while (value > cur && !target.compare_exchange_weak(cur, value)) {
    }
}
} // namespace

bool TopoOffsetTetMesh::smooth_before(const Tuple& t)
{
    ++m_smooth_trace.attempted;
    const size_t vid = t.vid(*this);
    // The final Phase A does not move the front: it is converged by then, its smoothing there
    // would be AMIPS alone with the front free anywhere inside the offset tube, and no Phase B
    // follows to put it back.
    if (m_freeze_front && m_vertex_extra[vid].m_is_on_offset) return false;

    // Diagnostic, recorded for every visit; only visits whose ring already holds a needle are
    // counted, and smooth_after() reads this back.
    auto& pre = m_needle_pre.local();
    pre = {ring_max_quality(vid), m_vertex_attribute[vid].m_posf};
    if (pre.first >= kNeedleQuality) ++m_needle_smooth_offered;

    // The base's smooth_before minus its bounding-box refusal, which is why this does not call
    // it: the base freezes every vertex on the domain wall. Here the wall is a region boundary
    // held in ambient's tag envelope like any other, so its vertices are smoothed and the
    // containment check decides whether the move survives. As in 2D.
    //
    // Rounding still has to happen, and its failure still refuses the move.
    const bool rounded_now = round(t);
    if (!m_vertex_attribute[vid].m_is_rounded && !rounded_now) {
        ++m_smooth_trace.before_unrounded;
        return false;
    }

    return true;
}

bool TopoOffsetTetMesh::smooth_after(const Tuple& t)
{
    const size_t vid = t.vid(*this);
    const auto& ve = m_vertex_extra[vid];

    // Diagnostic, the other half of smooth_before()'s record.
    {
        const auto& pre = m_needle_pre.local();
        if (pre.first >= kNeedleQuality) {
            ++m_needle_smooth_reached;
            const double after = ring_max_quality(vid);
            const double moved = (m_vertex_attribute[vid].m_posf - pre.second).norm();
            if (after < kNeedleQuality) ++m_needle_smooth_fixed;
            if (moved < 1e-12) ++m_needle_smooth_stationary;
            if (m_needle_smooth_reports.fetch_add(1) < 8) {
                logger().info(
                    "[needle-smooth #{}] vid {} ring max {:.6g} -> {:.6g} ({:.3g}x) | moved "
                    "{:.6g} | input {} offset {} region {} mask 0x{:x} | pos ({:.17g}, {:.17g}, "
                    "{:.17g})",
                    m_needle_smooth_reports.load(),
                    vid,
                    pre.first,
                    after,
                    after / std::max(pre.first, 1e-300),
                    moved,
                    ve.m_is_on_input,
                    ve.m_is_on_offset,
                    ve.m_is_on_region,
                    ve.m_boundary_mask,
                    m_vertex_attribute[vid].m_posf[0],
                    m_vertex_attribute[vid].m_posf[1],
                    m_vertex_attribute[vid].m_posf[2]);
            }
        }
    }
    if (ve.m_is_on_region) {
        ++m_smooth_trace.region_attempted;
    }
    if (ve.m_is_on_offset) {
        ++m_smooth_trace.offset_attempted;
    } else {
        ++m_smooth_trace.interior_attempted;
    }

    // The plastic medium: a vertex whose whole ring is plastic and which no envelope holds flows
    // under rest-shape AMIPS alone (see smooth_plastic_vertex). As in 2D.
    if (m_plastic_active && !ve.m_is_on_offset && !smoothing_containment_envelope(vid)) {
        bool all_plastic = true;
        for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
            if (!cell_is_plastic(tid)) {
                all_plastic = false;
                break;
            }
        }
        if (all_plastic) {
            const bool okp = smooth_plastic_vertex(t);
            ++m_smooth_trace.interior_attempted;
            return okp;
        }
    }

    // Phase B: a front vertex goes through the shared smoother -- same solver, line search and
    // accept tests as every other vertex -- with the offset's options: its objective carries the
    // offset terms (smoothing_extra_energy) and there is no quality veto, since a front vertex
    // must be able to worsen its ring on the way to the level set. Shape is Phase A's job, and
    // every other vertex in both phases is TetWild's smooth_after() unchanged.
    if (phase_places_front() && ve.m_is_on_offset) {
        const bool ok = smooth_front_vertex_phase_b(t);
        if (ok) ++m_smooth_trace.offset_accepted;
        return ok;
    }
    // Phase A is TetWild: the shared smoother, with the front held by m_offset_envelope and
    // carrying no offset term of its own.
    const double before = ve.m_is_on_offset ? band_vertex_residual(vid) : 0.;
    const bool ok = TetOptimizerMesh::smooth_after(t);
    if (!ve.m_is_on_offset) {
        return ok;
    }
    const double after = band_vertex_residual(vid);
    if (ok) ++m_smooth_trace.offset_accepted;

    const auto nano = [](double x) { return static_cast<long long>(std::min(x, 1e9) * 1e9); };
    m_smooth_trace.res_before_nano += nano(before);
    m_smooth_trace.res_after_nano += nano(after);
    atomic_max(m_smooth_trace.res_max_before_nano, nano(before));
    atomic_max(m_smooth_trace.res_max_after_nano, nano(after));
    return ok;
}

} // namespace wmtk::components::topological_offset
