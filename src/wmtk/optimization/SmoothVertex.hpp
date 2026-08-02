#pragma once

#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/EnvelopeEnergy.hpp>
#include <wmtk/optimization/solver.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>

#include <array>
#include <atomic>
#include <memory>
#include <vector>

namespace wmtk::optimization {

/**
 * @brief Why a smoothing attempt was refused, counted per pass.
 *
 * A smoothing pass that rejects most of what it tries looks identical from outside to one
 * that is converging, and the difference between "the envelope will not let this vertex
 * move" and "moving it inverts a tet" points at completely different fixes. Counting is
 * cheap and turns that question into a log line.
 */
struct SmoothRejectCounters
{
    std::atomic<size_t> already_inverted{0}; ///< incident tet inverted in floats on entry
    std::atomic<size_t> envelope{0}; ///< a surface triangle left the envelope
    std::atomic<size_t> inverted{0}; ///< the move inverted a tet (exact predicate)
    std::atomic<size_t> quality{0}; ///< the move made the worst incident tet worse
    std::atomic<size_t> accepted{0};

    void reset()
    {
        already_inverted = 0;
        envelope = 0;
        inverted = 0;
        quality = 0;
        accepted = 0;
    }

    std::string to_string() const
    {
        return fmt::format(
            "accepted {} | rejected: pre-inverted {}, envelope {}, inverted {}, quality {}",
            accepted.load(),
            already_inverted.load(),
            envelope.load(),
            inverted.load(),
            quality.load());
    }
};

/**
 * @brief Weights and policy for smooth_vertex_3d.
 *
 * The weights follow simwild's convention: `w_envelope = 1 - w_amips`, with w_amips small
 * (1e-4), so the envelope term dominates and the AMIPS term acts as a light quality
 * preference. The scale factors put the two on a comparable footing --- AMIPS is
 * dimensionless while the envelope energy is a squared distance, so s_envelope is normally
 * 1 / (diag * eps^2).
 */
struct SmoothVertexOptions
{
    double w_amips = 1e-4;
    double w_envelope = 1.0 - 1e-4;
    double s_amips = 1.0;
    double s_envelope = 1.0;

    /// Solve once with reciprocal weights before the weighted solve. Cheap warm-up that
    /// gets the vertex into the right basin before the envelope term dominates.
    bool two_stage = true;

    /**
     * Refuse a move that makes the worst incident tet worse.
     *
     * Applied to interior vertices always. Applying it to *surface* vertices as well is
     * what deadlocks a mesh whose surface vertices have drifted: the envelope pulls the
     * vertex back, that pull worsens some incident tet, the move is refused, and the vertex
     * never returns. Left false, matching simwild.
     */
    bool quality_veto_on_surface = false;
};

/**
 * @brief One Newton smoothing step for a single vertex, shared by the 3D applications.
 *
 * The envelope enters the objective as a squared-distance penalty rather than as a
 * projection applied afterwards, so a vertex is drawn back toward the surface gradually and
 * the line search refuses any step that leaves the envelope
 * (EnvelopeEnergy3D::is_step_valid). That is the difference from a hard `nearest_point`
 * snap, which moves the vertex the whole way in one jump and is therefore rejected exactly
 * when the vertex most needs moving.
 *
 * `Mesh` must provide, on top of what wmtk::TetMesh already gives:
 *   - m_vertex_attribute[vid].{m_pos, m_posf, m_is_on_surface}
 *   - m_tet_attribute[tid].m_quality
 *   - is_inverted_f(Tuple), is_inverted(Tuple), get_quality(Tuple)
 *   - std::shared_ptr<SampleEnvelope> smoothing_envelope(size_t vid) const
 *     (null is allowed and means "no envelope term for this vertex")
 *
 * @return true if the new position was accepted; false leaves it to the caller's rollback.
 */
template <class Mesh>
bool smooth_vertex_3d(
    Mesh& m,
    const typename Mesh::Tuple& t,
    const SmoothVertexOptions& opts,
    std::unique_ptr<polysolve::nonlinear::Solver>& solver,
    SmoothRejectCounters* counters = nullptr)
{
    using Tuple = typename Mesh::Tuple;

    const size_t vid = t.vid(m);
    auto& VA = m.m_vertex_attribute;
    auto& TA = m.m_tet_attribute;

    const auto locs = m.get_one_ring_tets_for_vertex(t);
    assert(!locs.empty());

    double max_quality = 0.;
    for (const Tuple& tet : locs) {
        max_quality = std::max(max_quality, TA[tet.tid(m)].m_quality);
        if (m.is_inverted_f(tet)) {
            // A neighbour that is not rounded can leave a tet inverted in floats even
            // though it is fine in exact arithmetic; there is nothing to optimize from.
            if (counters) ++counters->already_inverted;
            return false;
        }
    }

    // AMIPS wants each tet as 12 doubles with the moving vertex first, so that the solver
    // can overwrite the leading xyz.
    std::vector<std::array<double, 12>> assembles(locs.size());
    for (size_t i = 0; i < locs.size(); ++i) {
        std::array<size_t, 4> local_verts = m.oriented_tet_vids(locs[i].tid(m));
        local_verts = wmtk::orient_preserve_tet_reorder(local_verts, vid);
        for (int k = 0; k < 4; k++) {
            for (int j = 0; j < 3; j++) {
                assembles[i][k * 3 + j] = VA[local_verts[k]].m_posf[j];
            }
        }
    }

    if (!solver) {
        solver = create_basic_solver();
    }

    const double amips_w = opts.w_amips > 0 ? opts.s_amips * opts.w_amips : 1.0;
    auto amips_energy = std::make_shared<AMIPSEnergy3D>(assembles, amips_w);
    std::shared_ptr<polysolve::nonlinear::Problem> total_energy = amips_energy;

    auto solve = [&]() {
        VectorXd x = VA[vid].m_posf;
        try {
            solver->minimize(*total_energy, x);
        } catch (const std::exception&) {
            // polysolve reports a failed line search by throwing; the position it reached
            // is still the best it found, and the checks below decide whether to keep it.
        }
        VA[vid].m_posf = x;
    };

    const std::shared_ptr<SampleEnvelope> pull_env =
        VA[vid].m_is_on_surface ? m.smoothing_energy_envelope(vid) : nullptr;

    if (pull_env) {
        auto envelope_energy =
            std::make_shared<EnvelopeEnergy3D>(pull_env, opts.s_envelope * opts.w_envelope);

        if (opts.two_stage) {
            auto warmup = std::make_shared<EnergySum>();
            if (opts.w_amips > 0) warmup->add_energy(amips_energy, 1. / opts.w_amips);
            if (opts.w_envelope > 0) warmup->add_energy(envelope_energy, 1. / opts.w_envelope);
            total_energy = warmup;
            solve();
        }

        auto weighted = std::make_shared<EnergySum>();
        if (opts.w_amips > 0) weighted->add_energy(amips_energy);
        if (opts.w_envelope > 0) weighted->add_energy(envelope_energy);
        total_energy = weighted;
        solve();
    } else {
        solve();
    }

    // Containment: every surface triangle at this vertex must still be inside. Checked
    // against the containment envelope, which is not necessarily the one it was pulled to.
    const std::shared_ptr<SampleEnvelope> check_env =
        VA[vid].m_is_on_surface ? m.smoothing_containment_envelope(vid) : nullptr;
    if (check_env) {
        const simplex::SimplexCollection surf = m.get_surface_faces_for_vertex(vid);
        for (const simplex::Face& f : surf.faces()) {
            const std::array<Eigen::Vector3d, 3> face = {
                {VA[f.vertices()[0]].m_posf,
                 VA[f.vertices()[1]].m_posf,
                 VA[f.vertices()[2]].m_posf}};
            if (check_env->is_outside(face)) {
                if (counters) ++counters->envelope;
                return false;
            }
        }
    }

    // The rational position must be current before the exact inversion test.
    VA[vid].m_pos = to_rational(VA[vid].m_posf);

    double max_after_quality = 0.;
    for (const Tuple& loc : locs) {
        if (m.is_inverted(loc)) {
            if (counters) ++counters->inverted;
            return false;
        }
        const size_t tid = loc.tid(m);
        TA[tid].m_quality = m.get_quality(loc);
        max_after_quality = std::max(max_after_quality, TA[tid].m_quality);
    }

    if (!VA[vid].m_is_on_surface || opts.quality_veto_on_surface) {
        if (max_after_quality > max_quality) {
            if (counters) ++counters->quality;
            return false;
        }
    }

    if (counters) ++counters->accepted;
    return true;
}

} // namespace wmtk::optimization
