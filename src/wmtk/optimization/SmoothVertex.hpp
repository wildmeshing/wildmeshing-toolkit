#pragma once

#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/EnvelopeEnergy.hpp>
#include <wmtk/optimization/solver.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>

#include <array>
#include <atomic>
#include <limits>
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
 * dimensionless while the envelope energy is a squared distance, so s_envelope is
 * 1 / eps^2 and the term reads (d/eps)^2: distance in envelope widths, dimensionless
 * like AMIPS in both 2D and 3D.
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
     * Refuse a move that makes the worst incident element worse. Applied to every vertex.
     *
     * This used to be false for SURFACE vertices, on the grounds that vetoing them deadlocks
     * a mesh whose surface vertices have drifted: the envelope pulls such a vertex back, the
     * pull worsens some incident element, the move is refused, and the vertex never returns.
     *
     * That reasoning was sound but conditional -- it described a mesh built against an
     * envelope of eps/2, where surface vertices had drifted most of the way to the wall (a
     * population of ~100 sitting at 0.5-0.8 eps on Thingi10K 101954, never returning). With
     * the whole eps the drift is gone, the vertices start near the input, and the veto costs
     * them nothing.
     *
     * What it buys is large. Without it, smoothing raised the max energy in 38% of passes,
     * and a mesh that had descended to ~10.5 would then oscillate in a 10-13 band for tens
     * of iterations until a reading happened to fall below the target -- convergence by
     * random walk. Over 31 models (14 Thingi10K + 17 triwild20k, stop_energy 10) turning it
     * on cut total iterations from 691 to 350 with no model regressing by more than one
     * iteration and none left above target: triwild20k 166331 78 -> 13, 191265 58 -> 8,
     * 191874 58 -> 15, 202302 47 -> 12.
     *
     * A narrower form was tried and does not work: refusing only moves that create a new
     * GLOBAL worst element still lets a pass fill the mesh with elements just under the
     * current worst, which a later topology pass tips over. Bounding the maximum does not
     * bound the degradation (191265: 42 iterations against 58 baseline and 8 with the full
     * veto).
     */
    bool quality_veto_on_surface = true;

    /**
     * How a surface vertex is placed.
     *
     * Projected: smooth as if interior (AMIPS alone), then walk back toward the start along
     * t = 1, 1/2, 1/4, ..., projecting each candidate onto the input; take the first
     * projected candidate that does not invert and strictly lowers the worst incident
     * element, else do not move. The vertex lands exactly on the input; no weights exist.
     *
     * Exact: minimize w_amips * AMIPS + w_envelope * (d/eps)^2 with the TRUE Hessian of the
     * distance to the piecewise-linear input (ExactDistanceEnergy2D/3D): sliding is free
     * exactly where the input is flat, blocked isotropically at corners and vertices,
     * constrained to the direction of a 3D edge or curve segment. The vertex rests a
     * w_amips-proportional distance off the input; the accept checks below apply either way.
     */
    enum class SmoothingMode { Projected, Exact };
    SmoothingMode smoothing_mode = SmoothingMode::Projected;

    /// Backtracking steps tried before the move is abandoned: t = 1, 1/2, ..., 2^-(n-1).
    /// Exhausting these without an acceptable candidate is what "the search failed" means.
    int project_line_search_steps = 12;

    /**
     * Whether a smoothing move that raises the worst incident element's quality is refused at
     * all. True is TriWild's and SimWild's behaviour and the reasoning above applies to it.
     *
     * It stops applying when the objective is not the one that reasoning was written for. The
     * veto's premise is that the solver only ever asks for a SMALL move, so refusing costs
     * almost nothing and the next pass retries from the same place: a TriWild surface vertex
     * starts on the input and the envelope term keeps it there. topological_offset's offset
     * boundary is placed by minimising a term (see smoothing_extra_energy) whose minimum can be
     * most of the offset distance away, so a solved position routinely worsens some incident
     * element and would be refused. There the mesh's shape is the topological passes' job and
     * the smoother's job is to place the boundary, so the veto is turned off and the two are
     * left to their own work.
     *
     * False disables the check for EVERY vertex; quality_veto_on_surface only ever narrowed it.
     * Read by both smooth_vertex_2d and smooth_vertex_3d.
     */
    bool quality_veto = true;


    /**
     * Second pass, run only when the first found nothing: partial projections.
     *
     * The first pass only ever tests points ON the input, so a vertex whose one-ring cannot
     * tolerate being pulled all the way there is refused at every step and does not move at
     * all -- and is refused again next pass, from the same place. This pass revisits each
     * candidate and bisects between the interpolated point and its projection, taking the
     * longest step toward the input that still does not invert and still lowers the worst
     * element. The vertex ends up off the input, but closer to it than it was, so the next
     * pass starts from somewhere better and it converges onto the surface over several
     * passes instead of being stuck forever.
     *
     * 0 disables the pass. Costs nothing when the first pass succeeds, which is the common
     * case.
     */
    int project_line_search_nested_steps = 0;
};

/**
 * @brief One Newton smoothing step for a single vertex, shared by the 3D applications.
 *
 * The envelope enters the objective as a squared-distance penalty rather than as a
 * projection applied afterwards, so a vertex is drawn back toward the surface gradually and
 * a vertex is drawn back toward the surface gradually. That is the difference from a hard
 * `nearest_point` snap, which moves the vertex the whole way in one jump and is therefore
 * rejected exactly when the vertex most needs moving.
 *
 * The line search never consults the envelope: AMIPS keeps its pole guard against stepping
 * over an inversion, and eps-containment is owned by the accept checks after the solve,
 * which test whole incident faces rather than the vertex point anyway.
 *
 * `Mesh` must provide, on top of what wmtk::TetMesh already gives:
 *   - m_vertex_attribute[vid].{m_pos, m_posf, m_is_on_surface}
 *   - cell_quality(tid) / set_cell_quality(tid, quality)
 *   - is_inverted_f(Tuple), is_inverted(Tuple), get_quality(Tuple)
 *   - std::shared_ptr<SampleEnvelope> smoothing_energy_envelope(size_t vid) const
 *     and smoothing_containment_envelope(size_t vid) const
 *     (null is allowed and means "no envelope term for this vertex"; both dimensions)
 *   - std::shared_ptr<polysolve::nonlinear::Problem> smoothing_extra_energy(size_t vid) const
 *     (null is allowed and means "AMIPS plus the envelope, as before")
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
    const auto locs = m.get_one_ring_tets_for_vertex(t);
    assert(!locs.empty());

    double max_quality = 0.;
    for (const Tuple& tet : locs) {
        max_quality = std::max(max_quality, m.cell_quality(tet.tid(m)));
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

    // An application-supplied term for this vertex. It carries its own weight, exactly as
    // ExactDistanceEnergy3D below does, so it is summed here at 1. Null for TetWild and SimWild,
    // which leaves `base_energy` as `amips_energy` and every expression below exactly what it
    // was.
    const std::shared_ptr<polysolve::nonlinear::Problem> extra_energy =
        m.smoothing_extra_energy(vid);
    std::shared_ptr<polysolve::nonlinear::Problem> base_energy = amips_energy;
    if (extra_energy) {
        auto sum = std::make_shared<EnergySum>();
        if (opts.w_amips > 0) sum->add_energy(amips_energy);
        sum->add_energy(extra_energy);
        base_energy = sum;
    }
    std::shared_ptr<polysolve::nonlinear::Problem> total_energy = base_energy;

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

    if (pull_env && opts.smoothing_mode == SmoothVertexOptions::SmoothingMode::Projected) {
        // Smooth as if the vertex were interior, then walk back onto the input.
        const Vector3d x_orig = VA[vid].m_posf;
        total_energy = base_energy;
        solve();
        const Vector3d x_new = VA[vid].m_posf;

        // Place a candidate and report the worst incident quality, or infinity if it
        // inverts. is_inverted is exact, so the rational position tracks every candidate.
        const auto worst_at = [&](const Vector3d& p) {
            VA[vid].m_posf = p;
            VA[vid].m_pos = to_rational(p);
            double mq = 0.;
            for (const Tuple& loc : locs) {
                if (m.is_inverted(loc)) {
                    return std::numeric_limits<double>::infinity();
                }
                mq = std::max(mq, m.get_quality(loc));
            }
            return mq;
        };

        bool accepted = false;
        std::vector<Vector3d> interp, proj; // kept for the nested pass below
        interp.reserve(opts.project_line_search_steps);
        proj.reserve(opts.project_line_search_steps);
        for (int k = 0; k < opts.project_line_search_steps; ++k) {
            const Vector3d p = x_orig + std::pow(0.5, k) * (x_new - x_orig);
            Vector3d q;
            pull_env->nearest_point(p, q);
            interp.push_back(p);
            proj.push_back(q);
            if (worst_at(q) < max_quality) {
                accepted = true;
                break;
            }
        }

        // Nothing ON the input was acceptable anywhere, so settle for getting as close to
        // it as the one-ring allows. For each candidate, bisect the segment from the
        // interpolated point (s = 0) to its projection (s = 1) for the LARGEST acceptable s.
        // s = 1 is already known to fail -- that is what the first pass just established --
        // so this brackets the boundary and converges up to it from below. Halving s down
        // from 1 instead would cap the result at the midpoint and leave the vertex needlessly
        // far from the input.
        if (!accepted && opts.project_line_search_nested_steps > 0) {
            for (size_t k = 0; k < proj.size() && !accepted; ++k) {
                double lo = 0.0, hi = 1.0; // lo: best acceptable so far, hi: known bad
                Vector3d best;
                bool found = false;
                for (int j = 0; j < opts.project_line_search_nested_steps; ++j) {
                    const double mid = 0.5 * (lo + hi);
                    const Vector3d cand = interp[k] + mid * (proj[k] - interp[k]);
                    if (worst_at(cand) < max_quality) {
                        lo = mid;
                        best = cand;
                        found = true;
                    } else {
                        hi = mid;
                    }
                }
                if (found) {
                    // worst_at left the vertex at the last candidate tried, which is not
                    // necessarily the best one.
                    worst_at(best);
                    accepted = true;
                }
            }
        }
        if (!accepted) {
            VA[vid].m_posf = x_orig;
            VA[vid].m_pos = to_rational(x_orig);
            if (counters) ++counters->quality;
            return false;
        }
    } else if (pull_env) {
        auto envelope_energy =
            std::make_shared<ExactDistanceEnergy3D>(pull_env, opts.s_envelope * opts.w_envelope);

        if (opts.two_stage) {
            auto warmup = std::make_shared<EnergySum>();
            if (opts.w_amips > 0) warmup->add_energy(amips_energy, 1. / opts.w_amips);
            if (opts.w_envelope > 0) warmup->add_energy(envelope_energy, 1. / opts.w_envelope);
            if (extra_energy) warmup->add_energy(extra_energy);
            total_energy = warmup;
            solve();
        }

        auto weighted = std::make_shared<EnergySum>();
        if (opts.w_amips > 0) weighted->add_energy(amips_energy);
        if (opts.w_envelope > 0) weighted->add_energy(envelope_energy);
        if (extra_energy) weighted->add_energy(extra_energy);
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
        const double quality = m.get_quality(loc);
        m.set_cell_quality(tid, quality);
        max_after_quality = std::max(max_after_quality, quality);
    }

    if (opts.quality_veto && (!VA[vid].m_is_on_surface || opts.quality_veto_on_surface)) {
        if (max_after_quality > max_quality) {
            if (counters) ++counters->quality;
            return false;
        }
    }

    if (counters) ++counters->accepted;
    return true;
}


/**
 * @brief One Newton smoothing step for a single vertex of a 2D mesh.
 *
 * The 3D sibling above, one dimension down: 3-vertex elements assembled as 6 doubles, and
 * containment tested on the surface *edges* incident to the vertex rather than on triangles.
 * Deliberately a separate function rather than a dimension-templated one -- the arity, the
 * containment test and the position handling all differ, so branching on a dimension
 * parameter throughout would read worse than two bodies that each say what they do.
 *
 * `Mesh` must provide, on top of what wmtk::TriMesh already gives:
 *   - m_vertex_attribute[vid].m_is_on_surface, m_face_attribute[fid].m_quality
 *   - is_inverted_f(size_t fid), is_inverted(size_t fid), get_quality(size_t fid)
 *   - Vector2d smoothing_position(size_t vid) const
 *   - void set_smoothing_position(size_t vid, const Vector2d& p)
 *       writes the working position and its exact/rational copy together
 *   - m_envelope, used for both the surface pull energy and containment
 */
template <class Mesh>
bool smooth_vertex_2d(
    Mesh& m,
    const typename Mesh::Tuple& t,
    const SmoothVertexOptions& opts,
    std::unique_ptr<polysolve::nonlinear::Solver>& solver,
    SmoothRejectCounters* counters = nullptr)
{
    const size_t vid = t.vid(m);
    auto& VA = m.m_vertex_attribute;
    auto& FA = m.m_face_attribute;

    const std::vector<size_t>& locs = m.get_one_ring_fids_for_vertex(t);
    assert(!locs.empty());

    double max_quality = 0.;
    for (const size_t fid : locs) {
        max_quality = std::max(max_quality, FA[fid].m_quality);
        if (m.is_inverted_f(fid)) {
            // Nothing to optimize from: a neighbour that is not rounded can leave a face
            // inverted in floats even when it is fine exactly.
            if (counters) ++counters->already_inverted;
            return false;
        }
    }

    // AMIPS wants each face as 6 doubles with the moving vertex first, keeping the winding.
    std::vector<std::array<double, 6>> assembles;
    assembles.reserve(locs.size());
    for (const size_t fid : locs) {
        std::array<size_t, 3> vs = m.oriented_tri_vids(fid);
        size_t v_loc = 0;
        for (size_t i = 0; i < 3; ++i) {
            if (vs[i] == vid) {
                v_loc = i;
                break;
            }
        }
        const std::array<size_t, 3> buf = vs;
        vs[0] = buf[v_loc];
        vs[1] = buf[(v_loc + 1) % 3];
        vs[2] = buf[(v_loc + 2) % 3];

        std::array<double, 6> T;
        for (int i = 0; i < 3; i++) {
            const Vector2d p = m.smoothing_position(vs[i]);
            T[i * 2] = p[0];
            T[i * 2 + 1] = p[1];
        }
        assembles.push_back(T);
    }

    if (!solver) {
        solver = create_basic_solver();
    }

    const double amips_w = opts.w_amips > 0 ? opts.s_amips * opts.w_amips : 1.0;
    auto amips_energy = std::make_shared<AMIPSEnergy2D>(assembles, amips_w);

    // An application-supplied term for this vertex. It carries its own weight, exactly as
    // ExactDistanceEnergy2D below does, so it is summed here at 1. Null for TriWild and
    // SimWild, which leaves `base_energy` as `amips_energy` and every expression below exactly
    // what it was.
    const std::shared_ptr<polysolve::nonlinear::Problem> extra_energy =
        m.smoothing_extra_energy(vid);
    std::shared_ptr<polysolve::nonlinear::Problem> base_energy = amips_energy;
    if (extra_energy) {
        auto sum = std::make_shared<EnergySum>();
        if (opts.w_amips > 0) sum->add_energy(amips_energy);
        sum->add_energy(extra_energy);
        base_energy = sum;
    }
    std::shared_ptr<polysolve::nonlinear::Problem> total_energy = base_energy;

    auto solve = [&]() {
        VectorXd x = m.smoothing_position(vid);
        try {
            solver->minimize(*total_energy, x);
        } catch (const std::exception&) {
            // A failed line search is reported by throwing; the position reached is still
            // the best found, and the checks below decide whether to keep it.
        }
        m.set_smoothing_position(vid, Vector2d(x));
    };

    // Neighbours along the incident surface edges, captured before the solve. Only `vid`
    // moves, so their positions are the same either way, but taking them first matches what
    // both applications did and keeps the assert below meaningful.
    std::vector<Vector2d> surf_neighbors;
    if (VA[vid].m_is_on_surface) {
        const simplex::SimplexCollection es = m.get_surface_edges_for_vertex(vid);
        surf_neighbors.reserve(es.edges().size());
        for (const simplex::Edge& e : es.edges()) {
            const auto& evs = e.vertices();
            surf_neighbors.push_back(m.smoothing_position(evs[0] != vid ? evs[0] : evs[1]));
        }
        assert(!surf_neighbors.empty());
    }

    // Per-vertex rather than the mesh's single m_envelope, and split into the PULL and the
    // CONTAINER exactly as the 3D path above: see TriOptimizerMesh::smoothing_energy_envelope.
    // The defaults return the old expression, so TriWild and SimWild are unaffected.
    const std::shared_ptr<SampleEnvelope> envelope = m.smoothing_energy_envelope(vid);

    if (envelope && opts.smoothing_mode == SmoothVertexOptions::SmoothingMode::Projected) {
        const Vector2d x_orig = m.smoothing_position(vid);
        total_energy = base_energy;
        solve();
        const Vector2d x_new = m.smoothing_position(vid);

        // set_smoothing_position keeps the rational position in step, which the exact
        // is_inverted below depends on.
        const auto worst_at = [&](const Vector2d& p) {
            m.set_smoothing_position(vid, p);
            double mq = 0.;
            for (const size_t fid : locs) {
                if (m.is_inverted(fid)) {
                    return std::numeric_limits<double>::infinity();
                }
                mq = std::max(mq, m.get_quality(fid));
            }
            return mq;
        };

        bool accepted = false;
        std::vector<Vector2d> interp, proj; // kept for the nested pass below
        interp.reserve(opts.project_line_search_steps);
        proj.reserve(opts.project_line_search_steps);
        for (int k = 0; k < opts.project_line_search_steps; ++k) {
            const Vector2d p = x_orig + std::pow(0.5, k) * (x_new - x_orig);
            Vector2d q;
            envelope->nearest_point(p, q);
            interp.push_back(p);
            proj.push_back(q);
            if (worst_at(q) < max_quality) {
                accepted = true;
                break;
            }
        }

        // Nothing ON the input was acceptable anywhere, so settle for getting as close to
        // it as the one-ring allows. For each candidate, bisect the segment from the
        // interpolated point (s = 0) to its projection (s = 1) for the LARGEST acceptable s.
        // s = 1 is already known to fail -- that is what the first pass just established --
        // so this brackets the boundary and converges up to it from below. Halving s down
        // from 1 instead would cap the result at the midpoint and leave the vertex needlessly
        // far from the input.
        if (!accepted && opts.project_line_search_nested_steps > 0) {
            for (size_t k = 0; k < proj.size() && !accepted; ++k) {
                double lo = 0.0, hi = 1.0; // lo: best acceptable so far, hi: known bad
                Vector2d best;
                bool found = false;
                for (int j = 0; j < opts.project_line_search_nested_steps; ++j) {
                    const double mid = 0.5 * (lo + hi);
                    const Vector2d cand = interp[k] + mid * (proj[k] - interp[k]);
                    if (worst_at(cand) < max_quality) {
                        lo = mid;
                        best = cand;
                        found = true;
                    } else {
                        hi = mid;
                    }
                }
                if (found) {
                    // worst_at left the vertex at the last candidate tried, which is not
                    // necessarily the best one.
                    worst_at(best);
                    accepted = true;
                }
            }
        }
        if (!accepted) {
            m.set_smoothing_position(vid, x_orig);
            if (counters) ++counters->quality;
            return false;
        }
    } else if (envelope) {
        auto envelope_energy =
            std::make_shared<ExactDistanceEnergy2D>(envelope, opts.s_envelope * opts.w_envelope);

        if (opts.two_stage) {
            auto warmup = std::make_shared<EnergySum>();
            if (opts.w_amips > 0) warmup->add_energy(amips_energy, 1. / opts.w_amips);
            if (opts.w_envelope > 0) warmup->add_energy(envelope_energy, 1. / opts.w_envelope);
            if (extra_energy) warmup->add_energy(extra_energy);
            total_energy = warmup;
            solve();
        }

        auto weighted = std::make_shared<EnergySum>();
        if (opts.w_amips > 0) weighted->add_energy(amips_energy);
        if (opts.w_envelope > 0) weighted->add_energy(envelope_energy);
        if (extra_energy) weighted->add_energy(extra_energy);
        total_energy = weighted;
        solve();
    } else {
        solve();
    }

    // Per-vertex positional constraint, on top of the envelope. A mesh uses this to pin a
    // vertex to a 0-dimensional feature it stands for -- within a ball, so the vertex is
    // still free to move and improve quality, it just cannot walk away from the feature.
    // Meshes with no such features answer true unconditionally.
    if (!m.smoothing_position_is_allowed(vid, m.smoothing_position(vid))) {
        if (counters) ++counters->envelope;
        return false;
    }

    // Containment, edge by edge rather than face by face as in 3D -- and against the
    // CONTAINMENT envelope, which is not necessarily the one the vertex was pulled to.
    if (const std::shared_ptr<SampleEnvelope> hold_env = m.smoothing_containment_envelope(vid)) {
        const Vector2d p = m.smoothing_position(vid);
        for (const Vector2d& q : surf_neighbors) {
            const std::array<Eigen::Vector2d, 2> edge = {{p, q}};
            if (hold_env->is_outside(edge)) {
                if (counters) ++counters->envelope;
                return false;
            }
        }
    }

    double max_after_quality = 0.;
    for (const size_t fid : locs) {
        if (m.is_inverted(fid)) {
            if (counters) ++counters->inverted;
            return false;
        }
        const double q = m.get_quality(fid);
        FA[fid].m_quality = q;
        max_after_quality = std::max(max_after_quality, q);
    }

    if (opts.quality_veto && (!VA[vid].m_is_on_surface || opts.quality_veto_on_surface)) {
        if (max_after_quality > max_quality) {
            if (counters) ++counters->quality;
            return false;
        }
    }

    if (counters) ++counters->accepted;
    return true;
}

} // namespace wmtk::optimization
