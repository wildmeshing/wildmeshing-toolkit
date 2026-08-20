#pragma once

#include <array>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <mutex>

#include <wmtk/TetMesh.h>
#include <wmtk/TetOptimizerMesh.h>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include "OffsetPotential.hpp"
#include "Parameters.h"
#include "SimplicialComplexBVH.hpp"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

using CellTag = std::set<int64_t>;


namespace wmtk::components::topological_offset {

/// TEMPORARY, for the switch-isolation sweep. Reads one of the experiment switches from the
/// environment so all factors can be varied without a rebuild. `def` is ALWAYS the behavior the
/// branch had BEFORE the experiment, so an unset environment reproduces the original exactly and
/// nothing here changes the default build. Accepts 0/1, true/false, yes/no.
inline bool offset_experiment_flag(const char* name, const bool def)
{
    const char* v = std::getenv(name);
    if (v == nullptr || v[0] == '\0') return def;
    return !(v[0] == '0' || v[0] == 'f' || v[0] == 'F' || v[0] == 'n' || v[0] == 'N');
}


const int64_t TEMP_OFFSET_TET_TAG = -1;
const CellTag TEMP_OFFSET_TET_TAG_SET{TEMP_OFFSET_TET_TAG};


// for all attributes:
// label: 0=default, 1=input, 2=offset

/**
 * @brief Per-vertex data the shared optimizer knows nothing about.
 *
 * Position, rounding, bbox membership, order, sizing and partition all live on
 * wmtk::TetOptimizerMesh::VertexAttributes; this carries only what is the offset's own.
 * Registered with the base's m_vertex_attr_group, so it is resized, protected and rolled back
 * with the shared collection.
 *
 * Note m_is_on_input / m_is_on_offset versus the base's m_is_on_surface: the offset tracks TWO
 * surfaces, and the base's flag is their union. These two say which.
 */
class VertexExtra
{
public:
    int label = 0;
    size_t component_id = 0;
    bool m_is_on_input = false; // is this vertex on the input complex
    bool m_is_on_offset = false; // is this vertex on the offset surface

    /**
     * WHICH tag-region boundaries this vertex lies on, one bit per input tag (m_tag_bit).
     *
     * Seeded at init from the boundary faces themselves and then propagated like
     * m_is_on_input: a split midpoint takes the AND of its edge's endpoints (on both
     * boundaries only if the whole edge was), a collapse survivor ORs in the removed
     * vertex's bits (conservative, the same over-approximation the flags already accept).
     * A face's constraint mask is the AND of its three corners' masks -- a face lies on a
     * boundary only if all of it does -- which is what envelope_for_mask() dispatches on.
     * Never recomputed from current tet tags, so the band's tag rewriting during offset
     * construction cannot corrupt it.
     */
    uint64_t m_boundary_mask = 0;

    /**
     * WHICH SPLIT PASS CREATED THIS VERTEX, or 0 if no optimization split did.
     *
     * Churn instrumentation only -- nothing reads it to make a decision. Written by
     * split_after_cells() from TetOptimizerMesh::m_op_epoch and read by collapse_after_vertex()
     * on the vertex being REMOVED, so a split whose vertex is collapsed away again can be
     * counted, and counted separately by how long it survived. ASSIGNED rather than OR'd or
     * left alone, for the same reason m_boundary_mask is: v_id may be a recycled slot still
     * carrying a dead vertex's epoch, which would otherwise read as a very old birth.
     *
     * Zero means "not born of an optimization split": construction-era vertices are created
     * before any split pass has run, so m_op_epoch is still 0 for all of them.
     */
    uint32_t m_born_epoch = 0;
};


class EdgeAttributes
{
public:
    int label = 0; // label: 0=default, 1=input, 2=offset
};


/**
 * @brief Per-face data the shared optimizer knows nothing about.
 *
 * Whether a face is tracked surface, which surface, and which bbox side it is on all live on
 * wmtk::SurfaceTagAttributes. Only the construction label is the offset's own, and only the
 * pre-optimization phase reads it (marching tets, simplicial embedding, the offset growth in
 * Spatial.cpp) -- optimize_offset() consumes it once and ignores it from then on.
 *
 * Registered with the base's m_face_attr_group, so it survives the mesh changing shape. Its
 * VALUES, though, are only maintained by the offset's own operations: the shared split,
 * collapse and swap propagate face data by copying SurfaceTagAttributes around and never see
 * this. That is sound precisely because the label is dead by the time they run.
 */
class FaceExtra
{
public:
    int label = 0; // label: 0=default, 1=input, 2=offset
};


class TetAttributes
{
public:
    int label = 0; // label: 0=default, 1=input, 2=offset
    CellTag tag;
    double m_quality = 0; // AMIPS energy, kept up to date by smoothing
};


/**
 * @brief The offset's tet mesh, on the shared 3D optimizer.
 *
 * The construction phase -- marching tets, simplicial embedding, growing the offset region --
 * is entirely its own and uses TetMesh's face and tet splits directly. The optimization phase
 * that follows is wmtk::TetOptimizerMesh's, with the offset supplying only its own policy
 * through the hooks: which cell quality, which tags to propagate, which surface is
 * envelope-constrained, and how an offset-surface vertex is smoothed.
 *
 * Two surfaces are tracked, not one. The input complex is class 0 and must stay inside
 * m_envelope; the offset boundary is OFFSET_SURFACE_CLASS and is free to move, being defined by
 * the tet labels rather than by input geometry. The base's m_is_surface_fs / m_is_on_surface
 * are the UNION of the two -- that is what must not be torn by an operation -- and
 * VertexExtra::m_is_on_input / m_is_on_offset say which.
 */
class TopoOffsetTetMesh : public wmtk::TetOptimizerMesh
{
public:
    /**
     * @brief SurfaceTagAttributes::m_surface_class for the offset boundary.
     *
     * The input complex keeps the primary class 0, so a face carrying input geometry is
     * envelope-checked by the shared operations exactly as in tetwild and simwild.
     */
    static constexpr int OFFSET_SURFACE_CLASS = 1;

    /**
     * @brief Which half of the alternating optimization is running.
     *
     * THE TWO CRITERIA ARE OPTIMIZED IN TURN, NOT JOINTLY. Driving AMIPS and the Phi residual
     * from one loop put them in direct competition: split refined the band, the elements it
     * produced were badly shaped, collapse removed them on quality grounds, and the next
     * iteration started from where the last one did. Measured over 20 iterations on
     * topological_offset_3d_convex: the metric's best value came at iteration 5 and drifted
     * upward for the remaining fifteen, while every iteration ran split 945 -> 11973 vertices
     * followed by collapse 11973 -> 742.
     *
     * PHASE A is TetWild, and nothing else. Same operations, same gates, same sizing field,
     * same stall-driven refinement -- the offset contributes no energy term, no acceptance
     * criterion and no stop metric. Its one addition is m_offset_envelope: the offset surface
     * may not leave a tube of one Phi tolerance around wherever Phase B last placed it. That
     * turns "do not degrade the offset" from a per-operation criterion, which reached only the
     * 2% of collapses that touch the offset surface, into a geometric constraint every
     * operation already honours through surface_triangle_is_outside().
     *
     * PHASE B moves the offset surface and nothing else: smoothing passes against the offset
     * energy, run to a fixed point, with no envelope on the offset (it is what has to travel)
     * and no topological operations at all.
     *
     * The sizing field is SHARED and both phases write it: Phase A through TetWild's own
     * stall refinement on element quality, Phase B through the Phi residual of the faces that
     * smoothing could not place. One field, two reasons to refine it.
     */
    enum class OptPhase { A, B };

    /// Which phase is running. Read by every hook that differs between them; see OptPhase.
    OptPhase m_phase = OptPhase::A;


    /// DIAGNOSTIC: set by the driver after the first round when
    /// ab_no_collapse_after_first_round is on, and read by collapse_edge_before().
    bool m_ab_collapses_disabled = false;

    /**
     * @brief The tube the offset surface may not leave during Phase A. Null in Phase B.
     *
     * REBUILT AT THE START OF EVERY PHASE A, from the offset surface as Phase B just left it,
     * with eps = ab_offset_envelope_rel x the Phi tolerance. Rebuilding is what lets the
     * surface still travel across rounds: each Phase A pins it near its current position, and
     * each Phase B is free to move it somewhere the next Phase A will then pin.
     *
     * m_envelope, the INPUT complex's, is built once and never rebuilt -- that geometry is what
     * the offset distance is measured against and may not drift at all.
     */
    std::shared_ptr<SampleEnvelope> m_offset_envelope;

    /// Build m_offset_envelope from the current offset-surface triangles. Called at the start
    /// of each Phase A.
    void rebuild_offset_envelope();

    /// Hard error if any vertex is on BOTH the input complex and the offset surface -- a state
    /// no placement satisfies. Called at construction and after every phase; see the definition
    /// for why a collapse can create it out of two individually fine vertices.
    void check_no_vertex_on_both_surfaces(const char* when) const;

    /// The A/B driver: Phase A (TetWild + offset envelope), Phase B (smoothing to a fixed point,
    /// then refine where Phi is stuck), repeated until both criteria are inside tolerance or
    /// ab_max_rounds is reached. Replaces the single mesh_improvement() call.
    void optimize_offset_alternating();

    /// Phase B's smoothing loop. Returns the number of passes run; stops when the largest vertex
    /// displacement in a pass falls below ab_smooth_tol x l, or at ab_smooth_max_passes.
    size_t phase_b_smooth();

    /**
     * @brief L-inf over offset vertices of the gradient of each vertex's own placement
     * objective -- the same energy phase B's Newton solves minimize (w_amips * AMIPS over the
     * one-ring plus w_envelope * (Phi - c)^2). Exactly 0 at the Gauss-Seidel fixed point,
     * whatever the residual, so it distinguishes "placement finished" from "placement blocked"
     * -- a vertex whose move is refused contributes zero DISPLACEMENT but full gradient.
     * Vertices with a float-inverted or unrounded incident tet are skipped, as the smoother
     * skips them.
     */
    double phase_b_band_gradient_linf();

    /// Refine the shared sizing field around the offset faces that are still over tolerance
    /// after Phase B converged: the worst faces' one-rings have their sizing scalar scaled by
    /// stuck_refine_factor. Returns the number of vertices whose scalar was lowered.
    size_t refine_sizing_where_phi_is_stuck();

    /**
     * @brief Per-vertex band sizing update, run after Phase B. Returns the number changed.
     *
     * REFINE-ONLY, AND ONLY ON PURE CHORD ERROR. "In tolerance" is the CONVERGENCE CRITERION
     * itself -- |grad (Phi - c)^2| <= offset_gradient_tolerance() -- measured at band vertices
     * and at face-interior samples on the one lattice for_each_offset_face_sample() defines, so
     * this responds to exactly the quantity that decides the run rather than to a proxy for it.
     *
     *  - HALVE when the vertex and every surface one-ring neighbour are in tolerance but some
     *    sample inside an incident surface face is not. The surface passes through the right
     *    places and the triangle between them still cuts the level set: pure resolution error,
     *    the one thing a finer sizing field can actually fix.
     *  - Otherwise leave it alone. In particular a vertex that is itself out of tolerance is
     *    MISPLACED, not under-resolved, and refining around it just grows the mesh where Phase B
     *    has not finished; and a vertex in tolerance with an out-of-tolerance neighbour is that
     *    neighbour's problem to fix, not this one's.
     *
     * This is the chord-only trigger of refine_sizing_where_phi_is_stuck(), moved from a
     * per-FACE test with a 3-vertex condition to a per-VERTEX one with a surface one-ring
     * condition -- so a face is only refined when the whole neighbourhood around it is placed,
     * not merely its own corners.
     *
     * Clamped below by max(min_sizing_scalar, min_edge_length / l) -- the same band floor
     * refine_sizing_where_phi_is_stuck() uses -- and the changed vertices seed
     * gradation_smooth_sizing(), so a re-sized patch does not sit against an untouched one.
     */
    size_t update_band_sizing_from_tolerance();

    /**
     * @brief AVERAGE distance error, as a fraction of target_distance, above which a
     * non-converged run with respect_all_topologies is reported as topologically blocked.
     *
     * The discriminator is the AVERAGE, not the max and not max/avg. Topological blocking is
     * WIDESPREAD -- whole stretches of offset cannot advance, so the average climbs toward the
     * max. Every other failure mode is the opposite shape: a few over-constrained vertices pin
     * the max while the rest lands correctly, leaving the average tiny. In 2D avg/delta separated
     * the blocked runs from every other run by more than 150x.
     *
     * Class-scope rather than namespace-scope only because TopoOffsetTriMesh.h already defines a
     * namespace-scope constant of this name and the two headers share a namespace.
     */
    static constexpr double TOPOLOGY_BLOCK_AVG_FRAC = 0.05;

public: // mode for splitting in marching tets
    enum class EdgeSplitMode {
        Midpoint = 0, // used for simplicial embedding steps
        Initial = 1, // this is used to initialize the complex. Its a little hacky

        Optimization = 5 // this is used in the optimization phase of the algorithm
    };

public:
    int m_vtu_counter = 0;
    std::array<size_t, 4> m_init_counts = {{0, 0, 0, 0}};
    size_t m_tags_count;
    SimplicialComplexBVH m_input_complex_bvh;

    /**
     * @brief The smooth offset potential Phi of the input complex, as loaded.
     *
     * The offset surface is the level set Phi = c. Built from the SAME extraction as
     * m_input_complex_bvh, in the same call, so the two describe the same geometry. Like the
     * BVH, it is built once from the input and NEVER rebuilt: every offset quantity is measured
     * against the input as given, not against whatever the mesh has drifted to.
     *
     * shared_ptr because OffsetEnergy3D holds one per smoothing call.
     */
    std::shared_ptr<OffsetPotential3D> m_offset_potential;

    /**
     * @brief The EXACT-kind envelope of the input complex, built only for offset_field
     * "euclidean". Null otherwise.
     *
     * Not a containment envelope and never used as one: no operation tests against it. It exists
     * because nearest_point_feature() -- the foot point plus the feature kind the exact distance
     * derivatives are cased on -- is only answered by the exact path. Built from the same
     * extraction as m_input_complex_bvh, so it describes the same geometry.
     */
    std::shared_ptr<SampleEnvelope> m_input_complex_envelope;

    /**
     * @brief ONE CONTAINMENT ENVELOPE PER INPUT TAG, ambient included. Both phases.
     *
     * E_t is a tube of half-width m_envelope_eps around the boundary faces of region t as the
     * INPUT mesh carried them -- built in init_surfaces_and_boundaries(), before offset
     * construction, so the band's later tag rewriting never enters them. A simplex on several
     * boundaries is constrained by the INTERSECTION of its tags' tubes (envelope_for_mask()),
     * which pins junction curves and points to the junction itself -- including the input
     * complex's isolated wires and points, which only arise where two or more selected tags
     * meet. That intersection is what replaced the dedicated input-complex envelopes
     * (m_input_tri_env / m_input_seg_env) and their per-stratum dispatch: every simplex of the
     * complex lies on tag boundaries (see label_input_complex()), so the per-tag tubes already
     * hold all of it, and hold its junctions tighter than the old fused union-tube could.
     *
     * m_envelope (the base's pointer) survives as a UnionEnvelope over these members, purely so
     * the shared engine's one direct use -- the collapse_edge_before point check, union
     * semantics -- keeps working unchanged.
     *
     * INTERIOR VERTICES OF THE COMPLEX ARE NOT HELD BY THESE: a face interior to a region has
     * identical tag sets on both sides and lands in no bucket, so a tet-filled complex's
     * interior is free to optimise, exactly as before.
     */
    std::map<int64_t, std::shared_ptr<SampleEnvelope>> m_tag_envelopes;

    /// Input tag id -> bit position in VertexExtra::m_boundary_mask. Assigned in
    /// init_from_image() once the tag maps are complete; at most 64 input tags.
    std::map<int64_t, int> m_tag_bit;

    /// Memoized IntersectionEnvelope per multi-bit mask. Lazily built under the mutex because
    /// the queries that need them run concurrently under kPartition.
    mutable std::map<uint64_t, std::shared_ptr<SampleEnvelope>> m_isect_cache;
    mutable std::mutex m_isect_mutex;

    EdgeSplitMode m_edge_split_mode = EdgeSplitMode::Midpoint;

    // tag map stuff
    std::map<std::string, int64_t> m_tag_name_to_id;
    std::map<int64_t, std::string> m_tag_id_to_name;
    CellTag m_offset_output_tag_ids;

    // if in 'singlebody' mode
    bool m_singlebody = false;
    int64_t m_single_tag;

    // dont actually use, just for retaining in output
    bool m_has_envelope;
    std::vector<Vector3d> m_V_envelope;
    std::vector<Vector3i> m_F_envelope;
    // m_envelope itself lives on the base, which is what checks tracked-surface triangles
    // against it.
    double m_envelope_eps = -1;

    /// The base holds only wmtk::OptimizerParameters; this is the same object, typed, for the
    /// offset-only fields.
    Parameters& m_offset_params;

    using VertexExtraCol = wmtk::AttributeCollection<VertexExtra>;
    using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes>;
    using FaceExtraCol = wmtk::AttributeCollection<FaceExtra>;
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    // m_vertex_attribute and m_face_attribute are the base's; these two are registered
    // alongside them in its attribute groups.
    VertexExtraCol m_vertex_extra;
    FaceExtraCol m_face_extra;
    EdgeAttCol m_edge_attribute;
    TetAttCol m_tet_attribute;

    /**
     * @brief A face's shared surface tags together with the offset's own label.
     *
     * The marching-tets splits snapshot a face and write it back onto the pieces it became,
     * and both halves have to travel together.
     */
    struct FaceSnapshot
    {
        FaceAttributes tags;
        FaceExtra extra;
    };
    FaceSnapshot face_snapshot(const size_t fid) const
    {
        return FaceSnapshot{m_face_attribute[fid], m_face_extra[fid]};
    }
    void restore_face(const size_t fid, const FaceSnapshot& s)
    {
        m_face_attribute[fid] = s.tags;
        m_face_extra[fid] = s.extra;
    }

    /**
     * @brief Place a vertex, keeping its exact and rounded coordinates in step.
     *
     * The offset works in doubles throughout -- its construction is driven by a BVH distance
     * field to the input complex, which has no exact form -- so every vertex it places is
     * rounded, and its rational position is just the exact value of the double it carries.
     *
     * Filling m_pos anyway is not bookkeeping for its own sake. The shared split falls back to
     * the EXACT midpoint when the double one would invert an incident tet, which is what lets a
     * degenerate region keep being refined instead of the split being refused; that fallback
     * reads the endpoints' m_pos, and every quality and orientation test around an un-rounded
     * vertex reads its neighbours'. A vertex placed here with a stale m_pos would silently feed
     * those the wrong point.
     */
    void set_vertex_position(const size_t vid, const Vector3d& p)
    {
        m_vertex_attribute[vid].m_posf = p;
        m_vertex_attribute[vid].m_pos = to_rational(p);
        m_vertex_attribute[vid].m_is_rounded = true;
    }

    /**
     * @brief Whether tet `tid` belongs to the closed offset region, read from its TAGS.
     *
     * The 3D counterpart of TopoOffsetTriMesh::face_in_region, and it fixes the same defect.
     * The region is the offset band plus the input complex it wraps, both named by tags -- and
     * tags are what the shared operations propagate. The construction label says the same
     * thing but is derived state maintained alongside them, so it goes stale the moment a
     * split or swap creates a cell the label was never written for, putting holes in the
     * region that offset_is_manifold() then reports as non-manifold.
     */
    bool cell_in_region(const size_t tid) const
    {
        const int l = m_tet_attribute[tid].label;
        return l == 1 || l == 2; // input complex, or the offset band
    }

    /// Whether face `fid` is on the offset boundary (as opposed to the input complex).
    bool face_is_offset(const size_t fid) const
    {
        return m_face_attribute[fid].m_is_surface_fs &&
               m_face_attribute[fid].m_surface_class == OFFSET_SURFACE_CLASS;
    }
    /// Whether face `fid` carries input-complex geometry.
    bool face_is_input(const size_t fid) const
    {
        return m_face_attribute[fid].m_is_surface_fs &&
               m_face_attribute[fid].m_surface_class != OFFSET_SURFACE_CLASS;
    }

    // debug use
    std::atomic<int> cnt_split = 0, cnt_collapse = 0;

    TopoOffsetTetMesh(Parameters& _m_offset_params, int _num_threads = 0)
        : wmtk::TetOptimizerMesh(_m_offset_params, nullptr)
        , m_offset_params(_m_offset_params)
    {
        NUM_THREADS = _num_threads;
        // The base owns the vertex and face slots; register the offset's own data with its
        // groups so it is resized, protected and rolled back with them.
        m_vertex_attr_group.add(&m_vertex_extra);
        m_face_attr_group.add(&m_face_extra);
        p_edge_attrs = &m_edge_attribute;
        p_tet_attrs = &m_tet_attribute;

        m_collapse_check_link_condition = false;
        m_collapse_check_manifold = false;

        optimization::deactivate_opt_logger();
    }

    ~TopoOffsetTetMesh() override = default;

    ////// wmtk::TetOptimizerMesh hooks

    double cell_quality(const size_t tid) const override { return m_tet_attribute[tid].m_quality; }
    void set_cell_quality(const size_t tid, const double q) override
    {
        // SPIKE TRACKER. Every operation that writes a quality comes through here -- split, the
        // two collapse paths, all four swap paths, and smoothing -- so this is the one place that
        // sees a tet cross from ordinary into absurd. The passes log their own "==splitting 0=="
        // banners, so the spike's POSITION in the log names the operation with no plumbing.
        //
        // What it exists to explain: on specific_models/prism the max AMIPS jumps to ~1e16 during
        // the split pass and then to ~1e21 during the smoothing that follows, every iteration,
        // while the collapse pass afterwards returns it to ~1e3. avg goes to 5.7e12, so it is
        // thousands of tets, not one. Only the crossing is reported (was below, now above), and
        // only the first few in full, or the flood would be the whole log.
        if (m_spike_track) {
            const double before = m_tet_attribute[tid].m_quality;
            const double a = spike_amips(q), b = spike_amips(before);
            // TWO EVENTS, because the first cut at this logged neither of the ones that matter.
            //
            // Logging every crossing of a fixed bar spent the whole budget on the MILDEST tets --
            // measured, ten dumps all with `before` already at 7e4-1e5 while the pass max was
            // 4.6e16, so the extremes were never seen and nothing said where they came from.
            //
            // GENESIS: an ordinary tet ruined by ONE operation. This is the causal event.
            // RECORD:  a new global maximum, so the sequence of dumps ends at the true worst
            //          and the output stays bounded (records only increase).
            if (b < m_spike_ordinary && a > m_spike_threshold) {
                log_quality_spike(tid, before, q, "genesis");
            }
            if (a > m_spike_record) {
                m_spike_record = a;
                log_quality_spike(tid, before, q, "record");
            }
            if (a > m_spike_threshold) ++m_spike_count;
        }
        m_tet_attribute[tid].m_quality = q;
    }

    /// cell_quality is AMIPS^3; everything else reports its cube root, so convert once here.
    /// A non-finite quality is worse than any finite one, not silently zero.
    static double spike_amips(const double q)
    {
        return std::isfinite(q) ? std::cbrt(q) : std::numeric_limits<double>::infinity();
    }

    /**
     * @brief Report one tet crossing into absurd quality: its geometry, and why it is degenerate.
     *
     * Prints the four vertices with the two things that distinguish the plausible causes -- a
     * rounded/unrounded split (an unrounded vertex is exact-rational, and its float image can
     * make a perfectly valid tet look degenerate in doubles, which is why is_inverted_f and
     * is_inverted are separate predicates) and the tet's float volume against its edge lengths,
     * which separates a genuine sliver from a coordinate blow-up.
     */
    void log_quality_spike(size_t tid, double before, double after, const char* kind) const;

    /// Diagnostic: whether set_cell_quality reports tets crossing into absurd quality.
    bool m_spike_track = true;
    /// AMIPS above which a tet counts as absurd rather than merely bad.
    double m_spike_threshold = 1e5;
    /// AMIPS below which a tet counts as ORDINARY, so ordinary -> absurd is one operation's doing.
    /// stop_energy is 10 and the mesh average sits at 11-14, so 100 is comfortably "fine".
    double m_spike_ordinary = 100.;
    /// Largest AMIPS seen so far; a new maximum is logged, which bounds the output.
    mutable double m_spike_record = 0.;
    mutable std::atomic<int> m_spike_dumps{0};
    mutable std::atomic<int> m_spike_count{0};
    mutable std::atomic<int> m_spike_genesis{0};
    int m_spike_dump_budget = 24;

    /**
     * @brief Only the input complex is envelope-constrained.
     *
     * The offset boundary is defined by which tets carry the offset label, not by input
     * geometry, so there is nothing for it to stay inside; the checks that keep it faithful to
     * the implicit offset field are the normal-deviation ones in collapse and swap. Decided
     * from the vertices rather than the face's own class because the shared operations ask
     * about triangles they are creating, whose attributes are not written yet.
     */
    /**
     * @brief Is this vertex on a region boundary -- a tag boundary, or the domain wall.
     *
     * DERIVED, not stored. The two halves are already maintained exactly by the existing
     * machinery: m_is_on_input by is_edge_on_input() at each split and an OR at each collapse,
     * on_bbox_faces by set_intersection of the split endpoints and by the collapse rule that a
     * wall vertex may only merge into one at least as constrained. A stored third flag would be
     * a fourth thing to keep in step with those, for no information they do not already carry.
     */
    bool vertex_is_on_region(const size_t vid) const
    {
        return m_vertex_extra[vid].m_is_on_input || !m_vertex_attribute[vid].on_bbox_faces.empty();
    }

    /// The three helpers of the per-tag envelope dispatch. tag_bits() and face_mask() are
    /// trivial; envelope_for_mask() is out of line (it builds IntersectionEnvelopes lazily).
    uint64_t tag_bits(const CellTag& tags) const
    {
        uint64_t bits = 0;
        for (const int64_t t : tags) {
            const auto it = m_tag_bit.find(t);
            if (it != m_tag_bit.end()) bits |= (uint64_t(1) << it->second);
        }
        return bits;
    }

    /**
     * @brief The tag boundaries this vertex lies on -- the raw mask GATED on the vertex still
     * being region geometry at all.
     *
     * THE GATE IS NOT REDUNDANT, it is what keeps the mask honest. m_boundary_mask propagates
     * by a bare AND of a split's endpoints, which over-claims in exactly the way the flags do
     * not: an edge whose two ends happen to share a bit hands that bit to its midpoint even
     * when the edge itself is a chord through the interior, and the offset front is built by
     * splitting precisely such edges. Measured on prism before this gate: all 506 faces the
     * containment sweep called outside were offset faces carrying the ambient bit that way,
     * routed into a tube they sit a full target_distance from.
     *
     * vertex_is_on_region() is the predicate that does NOT over-claim -- m_is_on_input moves
     * across a split only through is_edge_on_input(), which demands a real incident input
     * face, and on_bbox_faces through the endpoints' intersection. So the mask says WHICH
     * boundaries, and this says whether the vertex is on one at all; the mask only ever
     * narrows an answer the flags already allow.
     */
    uint64_t vertex_boundary_mask(const size_t vid) const
    {
        return vertex_is_on_region(vid) ? m_vertex_extra[vid].m_boundary_mask : uint64_t(0);
    }

    /// A face lies on a boundary only if ALL of it does: the AND of its corners' masks. The
    /// same all-three shape the flag-based dispatch always had.
    uint64_t face_mask(const std::array<size_t, 3>& vids) const
    {
        return vertex_boundary_mask(vids[0]) & vertex_boundary_mask(vids[1]) &
               vertex_boundary_mask(vids[2]);
    }

    /**
     * @brief The envelope a simplex with this boundary mask is contained in, or null.
     *
     * Zero bits: no boundary, no container. One bit: that tag's own envelope. Several bits: a
     * memoized IntersectionEnvelope over the members -- inside means inside EVERY tube, which
     * pins junction geometry to the junction. CONTAINMENT-ONLY for the multi-bit case: the
     * composite implements just the virtual is_outside queries, so it must never be returned
     * from smoothing_energy_envelope() (the pull calls non-virtual nearest_point).
     */
    std::shared_ptr<SampleEnvelope> envelope_for_mask(uint64_t mask) const;

    std::shared_ptr<SampleEnvelope> surface_envelope_for_face(
        const std::array<size_t, 3>& vids) const override
    {
        // BOUNDARY GEOMETRY FIRST, in BOTH phases: a face on any tag-region boundary may not
        // drift out of that boundary's tube, and a face on several boundaries -- a junction
        // sliver -- is held in their intersection. The mask carries the input complex too:
        // every complex simplex lies on tag boundaries (label_input_complex() can only label
        // an isolated simplex whose tet star is tag-heterogeneous), so the per-tag tubes
        // subsume the deleted input-complex envelopes, as-loaded geometry and all -- E_t is
        // built from the INPUT mesh before construction touches it.
        if (const uint64_t mask = face_mask(vids)) {
            return envelope_for_mask(mask);
        }
        // Phase A holds the offset where Phase B left it; Phase B is what moves it, so it is
        // unconstrained there. Null when there is no offset envelope yet -- the construction
        // phase runs before the first one is built.
        bool all_offset = true;
        for (const size_t v : vids) {
            all_offset = all_offset && m_vertex_extra[v].m_is_on_offset;
        }
        if (all_offset && m_phase == OptPhase::A) return m_offset_envelope;
        return nullptr;
    }

    /// Surface edges may be flipped, as a topology-preserving diagonal flip. Both tracked
    /// surfaces need it: the offset boundary is re-triangulated constantly, and refusing to
    /// flip its diagonals is what leaves the badly-shaped triangles the sizing field then
    /// chases.
    bool allow_surface_swap() const override { return true; }
    bool check_surface_topology() const override { return m_offset_params.perform_sanity_checks; }

    /**
     * @brief The offset surface is the one tracked surface with NO envelope, in either role.
     *
     * It is the surface the optimization exists to move: a tube around wherever conservative
     * growth left it would cap how far it can ever travel toward the level set. What holds it
     * is the offset term in the OBJECTIVE -- smoothing_extra_energy() below -- not a container.
     *
     * The 2D twin returns the base's answer for every other vertex because 2D tracks
     * region-boundary curves as well; 3D tracks only these two.
     *
     * THE PULL MUST BE A REAL ENVELOPE, NEVER A COMPOSITE. This hook's consumers call the
     * NON-virtual SampleEnvelope queries -- nearest_point (projected smoothing) and the
     * ExactDistanceEnergy3D trio (squared_distance / nearest_point / nearest_point_feature) --
     * which on a composite would bind to the base's null BVH. So a junction vertex (several
     * mask bits) is pulled toward its MOST-VIOLATED member tube instead: one real envelope per
     * smoothing attempt, alternating projections toward the junction across passes, while the
     * containment intersection (smoothing_containment_envelope) enforces the full constraint.
     */
    std::shared_ptr<SampleEnvelope> smoothing_energy_envelope(const size_t vid) const override
    {
        if (m_vertex_extra[vid].m_is_on_offset && !vertex_is_on_region(vid)) {
            return nullptr;
        }
        const uint64_t mask = vertex_boundary_mask(vid);
        if (mask == 0) {
            // Reachable only for a construction artefact (a wall-chord midpoint flagged
            // on-surface with disjoint endpoint masks); its containment is vacuous too, so no
            // pull is behavior-neutral. Not an error.
            return nullptr;
        }
        std::shared_ptr<SampleEnvelope> best;
        double worst_d2 = -1.;
        for (const auto& [tag, env] : m_tag_envelopes) {
            const auto it = m_tag_bit.find(tag);
            if (it == m_tag_bit.end() || !(mask & (uint64_t(1) << it->second))) continue;
            if (!best) {
                best = env;
                if ((mask & (mask - 1)) == 0) break; // single bit: no violation contest to run
                worst_d2 = env->squared_distance(m_vertex_attribute[vid].m_posf);
                continue;
            }
            const double d2 = env->squared_distance(m_vertex_attribute[vid].m_posf);
            if (d2 > worst_d2) {
                worst_d2 = d2;
                best = env;
            }
        }
        return best;
    }

    /**
     * @brief ... and it is not CONTAINED by one either.
     *
     * The base's default answers m_envelope for every vertex the union flag m_is_on_surface is
     * set on, and here that flag covers both tracked surfaces. m_envelope is built from the
     * INPUT complex's boundary, so answering it for an offset-surface vertex would require the
     * offset to stay within eps of the input -- a tube of the wrong radius around the wrong
     * surface, which would refuse every move the offset term asks for.
     *
     * The INPUT COMPLEX falls through to the base and is contained by m_envelope, in every
     * phase. That is the whole constraint on it: it is smoothed like any other TetWild surface
     * vertex (see smooth_before()) and this tube is the tolerance it may drift within. An
     * earlier version of this comment said its vertices were "frozen anyway, so this only ever
     * answers for a vertex on both surfaces at once" -- both halves are now wrong. Freezing was
     * the pre-refactor behaviour and is gone, so this answers for EVERY input-complex vertex;
     * and a vertex on both surfaces at once is a construction defect that
     * check_no_vertex_on_both_surfaces() throws on, so that is the one case it never answers for.
     *
     * ... except in PHASE A, where it is contained by m_offset_envelope like any other tracked
     * surface. Phase A is TetWild and its smoothing minimises AMIPS alone; without a container
     * nothing would stop it relocating the offset surface for the sake of element shape, which
     * is precisely what Phase B then has to undo. Phase B keeps the original answer -- null --
     * because that is the pass whose whole job is to move the surface.
     */
    std::shared_ptr<SampleEnvelope> smoothing_containment_envelope(const size_t vid) const override
    {
        if (m_vertex_extra[vid].m_is_on_offset && !vertex_is_on_region(vid)) {
            return m_phase == OptPhase::A ? m_offset_envelope : nullptr;
        }
        // Boundary geometry is contained in the intersection of its tags' tubes, in BOTH
        // phases -- the caller only asks is_outside(triangle), which composites answer, so
        // unlike the pull this side may hand out an IntersectionEnvelope. Vertices with no
        // boundary bits (offset handled above; construction artefacts) have nothing to be
        // contained in.
        return envelope_for_mask(vertex_boundary_mask(vid));
    }

    /**
     * @brief The offset term, for a vertex on the offset surface; null for everything else.
     *
     * THIS IS WHERE THE OFFSET IS PLACED. w (Phi - c)^2, minimised by the shared smoother
     * alongside AMIPS -- so the offset surface is subject to the same line search, the same
     * exact inversion test and the same accept checks as every other vertex in the mesh, which
     * the quadrics/Laplacian blend that used to place it was not.
     *
     * A vertex that is also on the INPUT COMPLEX is excluded. It sits at distance 0 from the
     * complex by definition, where Phi diverges; asking it to reach the level set would be
     * asking it to leave the geometry the offset is measured from. (It never gets here anyway --
     * smooth_before() refuses it -- but the exclusion is stated where the reason lives.)
     */
    // NO smoothing_extra_energy OVERRIDE. The base's nullptr is correct for both phases now.
    //
    // This used to hand Phase B an OffsetEnergy3D so the offset term was minimised by the shared
    // AMIPS solver. Phase B no longer uses that solver at all: offset-only vertices go through
    // smooth_offset_vertex_backtracking()'s 1-D root find, and smooth_before() refuses every
    // other vertex in the phase. Phase A carries no offset term either -- it is TetWild, and the
    // offset surface is held there by m_offset_envelope, not by an energy. So the hook has no
    // caller left in either phase. OffsetEnergy3D itself survives; phase_b_band_gradient_linf()
    // still builds one to measure the placement gradient.

    /// The driver hands a bare `debug_N`; put the frames beside the run's own output instead of
    /// in whatever directory it happened to be launched from. Same as 2D's
    /// write_smoothing_debug_output(), and what lets the viewer find them as a timeline -- it
    /// globs `*debug_*.vtu` in the output directory and orders them by the counter.
    void write_optimization_debug_output(const std::string& path) override
    {
        // The per-pass debug_{N} series is opt-in: the driver's phase_{round}{A|B} frames are
        // the timeline the viewer shows, and the per-pass frames cost more than the rest of
        // the run put together (measured on prism: 801 frames, 3.95 GB, ~54s of 227s).
        if (!m_offset_params.debug_output_per_pass && path.rfind("debug_", 0) == 0) {
            return;
        }
        write_vtu(m_offset_params.output_path + "_" + path);
    }

    /**
     * @brief The engine's stall-driven sizing refinement, driven by the offset's criterion.
     *
     * mesh_improvement() fires this when optimization_quality_stats()'s max stalls; the
     * override ratchets the sizing down around the worst-scoring offset-surface FACES only
     * (TetWildMesh::refine_sizing_around_worst with face_criterion_rel() in place of AMIPS).
     */
    size_t refine_sizing_around_worst(double) override;

    /// No. The offset surface is the thing being placed and has no envelope holding it, so a
    /// bare collapse pass -- one not interleaved with splits and smoothing, and answering to no
    /// criterion -- can only decimate it. See the base for the measurement.
    bool optimization_bare_coarsen_passes() const override { return false; }

    /**
     * @brief (max, avg) Phi residual over the reachable offset surface, in units of its
     * tolerance; the engine's stop metric is therefore 1.0.
     */
    std::tuple<double, double> optimization_quality_stats() override;

    /// Phase A is TetWild, so it stops where TetWild stops: absolute AMIPS against stop_energy,
    /// which is the base's default. Phase B's metric is the Phi residual normalized by its own
    /// tolerance, so 1.0 means converged. The two must agree with optimization_quality_stats()
    /// above -- a metric in one unit tested against a bar in another is what stalled Phase A.
    double optimization_stop_metric() const override
    {
        return m_phase == OptPhase::A ? wmtk::TetOptimizerMesh::optimization_stop_metric() : 1.;
    }

    /// The tolerance the Phi residual is measured against: a fraction of target_distance, so
    /// "how close is close enough" is stated in units of the offset the run asked for.
    double offset_residual_tolerance() const
    {
        return std::max(
            m_offset_params.offset_residual_rel * m_offset_params.target_distance,
            1e-16);
    }

    /**
     * @brief THE CONVERGENCE TOLERANCE: the bound on |grad (Phi - c)^2| at a band vertex.
     *
     * A fraction of target_distance, which is the right unit: grad E = 2 (Phi - c) grad Phi, and
     * grad Phi is dimensionless for a field whose value is a length, so grad E is a length.
     *
     * WHY THIS REPLACED THE RESIDUAL BOUND. The residual is only comparable to target_distance
     * because residual_length() converts Phi's value into a length -- a conversion each potential
     * has to supply, and one that is only unambiguous where the level set is smooth and the
     * closest feature unique. The gradient needs no such conversion: it is the stationarity
     * condition of the objective Phase B minimises, so it is the same test for the exact
     * Euclidean field, for the OGC rule the smooth potential actually implements, and for ESP.
     * That is what lets a concave input be judged by the same number as a convex one.
     *
     * Not a restatement of the old bound. On a distance-like field the two coincide up to the
     * factor 2 (see offset_gradient_rel's default), but where |grad Phi| departs from 1 -- near a
     * medial axis, at a sharp feature, anywhere ESP's several active primitives compete -- they
     * are different tests and neither implies the other.
     */
    double offset_gradient_tolerance() const
    {
        return std::max(
            m_offset_params.offset_gradient_rel * m_offset_params.target_distance,
            1e-16);
    }

    /**
     * @brief Stop the run if any reachable offset-surface vertex has left the potential's support.
     *
     * Beyond dhat, Phi is identically zero WITH a zero gradient: the vertex is given no direction
     * back, its residual saturates instead of growing, and the sizing field refines around a
     * vertex nothing can move. There is no recovery from that state and no honest report of it
     * either, so it is a hard error rather than a warning -- if it turns out to fire on real
     * inputs, the answer is a larger offset_dhat_factor, not a quieter log line.
     *
     * Called once per optimization iteration, and once on the band as constructed.
     */
    void check_offset_within_support(const char* when) const;

    /// How far vid is from the level set Phi = c, as a length. This is what the loop converges
    /// on and what the sizing field refines by.
    double band_vertex_residual(const size_t vid) const;

    /// Whether vid is an offset-surface vertex the optimizer could still place on the level set.
    /// A vertex on the input complex sits where Phi diverges; one on the domain boundary is
    /// frozen there because conservative growth ran out of room. Neither is fixable -- but both
    /// still COUNT toward the residual (see residual_split()); this classifies, it no longer
    /// excludes.
    bool band_vertex_is_reachable(const size_t vid) const
    {
        return !m_vertex_extra[vid].m_is_on_input && m_vertex_attribute[vid].on_bbox_faces.empty();
    }

    /// Which vertices lie on the offset surface. Shared by every measurement so they all agree
    /// on what "the band" is.
    std::vector<bool> band_vertex_mask() const;

    /// A quantity sampled at points INSIDE an offset-surface face -- see offset_face_samples()
    /// for the residual and gradient_split() for the gradient.
    struct FaceSamples
    {
        double max = 0.;
        double sum = 0.;
        size_t n = 0;
    };

    /**
     * @brief The interior lattice a face is sampled on, handed to `visit` one point at a time.
     *
     * ONE DEFINITION OF "INSIDE THIS FACE", because two things are now measured across a face:
     * the Phi residual (the diagnostic) and the placement gradient (the criterion). Sampling
     * them on different lattices would make them incomparable -- the log prints one as a
     * multiple of the other's bar, and a run that fails the criterion in a face has to be
     * explicable by the residual measured at the same points.
     *
     * Interior lattice points of the barycentric subdivision at denominator n = k + 2: every
     * (i, j, l) with i + j + l = n and each >= 1, so nothing lands on an edge or a vertex (those
     * are measured separately, and a sample ON a vertex would double-count it).
     *
     * n = k + 2 rather than 2D's k + 1 because a triangle has no interior lattice point at
     * denominator 2. The counts are 1, 3, 6, 10 for k = 1, 2, 3, 4, so k keeps its meaning as
     * "sampling density" and k = 1 is the centroid -- which is the sample that matters most,
     * since a triangle inscribed in the offset is furthest from it at its centroid.
     */
    template <typename Visit>
    void for_each_offset_face_sample(const Tuple& f, Visit&& visit) const
    {
        const int k = m_offset_params.offset_residual_samples;
        if (k <= 0) return;

        const auto vs = get_face_vids(f);
        const Vector3d p0 = m_vertex_attribute[vs[0]].m_posf;
        const Vector3d p1 = m_vertex_attribute[vs[1]].m_posf;
        const Vector3d p2 = m_vertex_attribute[vs[2]].m_posf;

        const int n = k + 2;
        for (int i = 1; i < n; ++i) {
            for (int j = 1; j < n - i; ++j) {
                const int l = n - i - j;
                if (l < 1) continue;
                visit(Vector3d((double(i) * p0 + double(j) * p1 + double(l) * p2) / double(n)));
            }
        }
    }

    /**
     * @brief The Phi residual at `offset_residual_samples` interior points of offset face `f`.
     *
     * THE CRITERION CANNOT BE A VERTEX CRITERION. Distance error at the vertices alone does not
     * pin down the offset: a surface can have every vertex exactly on the level set while its
     * triangles cut across it, which reads as converged and is not the offset. That is the same
     * gap the paper's normal-deviation criterion (Sec. 5.3.3) covered, and it is not
     * hypothetical -- measured in 2D on topological_offset_2d_vertex_input, a band that had
     * decimated to twelve segments reported a vertex error of 0.2% of target_distance while its
     * edge midpoints sat at 18%; on the dragon, 5.5% at the vertices against 26% between them.
     *
     * Sampling the face INTERIOR is what the potential makes possible and the old distance field
     * did not: Phi is defined everywhere, so the offset can be measured anywhere on the surface
     * rather than only where the mesh happens to have put a vertex. The same samples feed
     * face_criterion_rel(), so the sizing field refines a surface too coarse to represent the
     * offset instead of letting it decimate -- which is what allowed the per-operation
     * normal-deviation guards in collapse and swap to be deleted.
     *
     * Returns nothing for a face with an UNREACHABLE corner: a triangle running onto the input
     * complex is legitimately closer than target_distance across part of itself, and no
     * operation can change that.
     */
    FaceSamples offset_face_samples(const Tuple& f) const;

    /// The per-face score refine_sizing_around_worst ranks by; >= 1 means the face fails the
    /// criterion. Also the per-face form of optimization_quality_stats().
    double face_criterion_rel(const Tuple& f) const;
    /// AMIPS of a cell over stop_energy -- the 3D twin of TriOptimizerMesh::quality_rel(), so
    /// both dimensions express "how bad is this element" on the same 1.0 scale.
    double cell_quality_rel(const size_t tid) const;
    /// ... and the worst of the (up to two) cells a face separates.
    double amips_rel_at_face(const Tuple& f) const;

    /**
     * @brief The offset surface's residual. Every band vertex and every face sample counts
     * toward the max and the average -- pinned vertices included. The reachable/pinned split
     * (see band_vertex_is_reachable()) is attribution: when the driving max comes from a
     * pinned vertex, the report can say so, and the remedy is construction rather than more
     * optimization.
     */
    struct DistanceSplit
    {
        double max_reachable = 0., avg_reachable = 0.;
        double max_pinned = 0.;
        size_t n_reachable = 0, n_pinned = 0;
        /// max_reachable, split by where it was measured. The two answer different questions:
        /// the vertex max says the surface is in the wrong PLACE, the face max says it is too
        /// COARSE to be in the right place -- and they call for different remedies (smoothing
        /// vs refinement), so a run that is not converging needs to know which it is.
        double max_at_vertex = 0., max_in_face = 0.;
        /// The runaway guard's tally, collected in the same traversal -- see
        /// check_offset_within_support() and report_outside_support().
        size_t n_outside_support = 0;
        size_t worst_outside_vid = static_cast<size_t>(-1);
        double worst_outside_dist = 0.;
    };
    DistanceSplit residual_split() const;

    /**
     * @brief The band's placement gradient: |grad (Phi(x) - c)^2| over the offset surface --
     * at every band vertex AND at interior samples of every band face.
     *
     * The convergence criterion. Structured like DistanceSplit, and split the same three ways,
     * because the questions a non-converging run asks are the same ones:
     *
     *  - reachable vs pinned -- only the reachable half gates the run; a pinned vertex with a
     *    large gradient is a construction defect and is reported rather than iterated on.
     *  - measured vs SKIPPED -- a vertex whose ring is already inverted, or which is not
     *    rounded, is one the smoother refuses to place, so its gradient is not part of the
     *    fixed point. Counted and logged rather than silently dropped: a run that "converges"
     *    with vertices it never measured is not a converged run.
     *
     *  - at-vertex vs IN-FACE. E = (Phi(x) - c)^2 is a field, not a mesh quantity: it has a
     *    gradient at every point of space, so it can be evaluated at points INSIDE a face just
     *    as the residual is. Both count toward max_reachable, so a triangle whose corners sit on
     *    the level set while its interior chords across it fails the criterion -- which is the
     *    whole reason the residual samples faces, and there is no reason the criterion should be
     *    the weaker of the two measurements. The split is kept because the two call for
     *    different remedies: at-vertex wants smoothing, in-face wants refinement.
     *
     * The in-face term is what an optimization step cannot directly reduce -- a face sample is
     * not a variable -- so it is the CONVERGENCE test that carries it and the Phase B stop test
     * that does not. See phase_b_band_gradient_linf().
     */
    struct GradientSplit
    {
        double max_reachable = 0., avg_reachable = 0.;
        double max_pinned = 0.;
        size_t n_reachable = 0, n_pinned = 0;
        /// max_reachable, split by where it was measured -- see the class comment.
        double max_at_vertex = 0., max_in_face = 0.;
        /// How many of n_reachable came from face interiors rather than vertices.
        size_t n_face_samples = 0;
        /// Band vertices the smoother would refuse to place, so their gradient is not part of
        /// the fixed point this measures. See smooth_offset_vertex_backtracking()'s entry guard.
        size_t n_skipped_inverted = 0, n_skipped_unrounded = 0;
        /// Where the driving max came from, for the log. worst_vid is set only when the max is
        /// an at-vertex one; an in-face max has no vertex to name.
        size_t worst_vid = static_cast<size_t>(-1);
    };
    /// @param include_face_samples false measures VERTICES ONLY -- what Phase B's stop test
    /// wants, since a face sample is not a variable its sweeps can move. The convergence test
    /// always passes true. See phase_b_band_gradient_linf().
    GradientSplit gradient_split(bool include_face_samples = true) const;
    /// Turn a residual_split()'s outside-support tally into the hard error. Separate from
    /// check_offset_within_support() so the per-iteration check can reuse a split it already has.
    void report_outside_support(const char* when, const DistanceSplit& s) const;

    /**
     * @brief Which tag the tets a swap creates should carry.
     *
     * A swap must not move the boundary between differently tagged regions, since that
     * boundary IS the offset. Around an interior edge every incident tet already shares a tag
     * (a face between differently tagged tets is a tracked surface face, and the base only
     * takes this path when the edge has none), so this is a cheap safety net there. On a
     * surface flip the ring genuinely spans two tags and the majority one wins.
     *
     * Returns false, refusing the swap, when three or more tags meet: there is then no single
     * answer and any choice would relabel a tet.
     */
    bool swap_before_interior(const std::vector<size_t>& tids) override;
    bool swap_before_surface(
        const std::vector<size_t>& tids,
        size_t a,
        size_t b,
        size_t c,
        size_t d) override;
    bool swap_after_cells(const std::vector<size_t>& tids, bool is_surface_flip) override;

    /**
     * @brief Collapse policy that is the offset's own.
     *
     * collapse_before_vertex refuses to move a vertex off the surface it belongs to (the base
     * only knows the union of the two), to lower the order of a boundary vertex, or to collapse
     * across what looks like a feature of the input complex; it also records how badly aligned
     * the offset surface already was, so its `after` counterpart can tell a regression from a
     * defect that was there before.
     */
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_before_vertex(size_t v1, size_t v2, double edge_length) override;
    bool collapse_after_connectivity(
        size_t v1,
        size_t v2,
        const std::vector<std::array<size_t, 2>>& boundary_edges) override;
    bool collapse_is_order_2_edge(const std::array<size_t, 2>& e) override
    {
        return is_order_2_edge(e);
    }
    void collapse_after_vertex(size_t v1, size_t v2) override;

    /// min: a collapse must not un-refine the sizing field -- see the base declaration.
    double collapse_merged_sizing(double removed, double survivor) const override
    {
        return std::min(removed, survivor);
    }

    /**
     * @brief Split policy that is the offset's own.
     *
     * The shared split places the vertex, keeps the quality and the shared attributes and
     * checks containment; what it cannot know is which region tag the two child tets inherit,
     * and which of the two tracked surfaces the new vertex joined.
     */
    bool split_before_cells(const Tuple& edge, const std::vector<Tuple>& parents) override;
    bool split_after_cells(size_t v1, size_t v2, size_t v_new, const std::vector<Tuple>& children)
        override;
    /// Writes the new vertex's tracked-surface membership before the shared split's containment
    /// check reads it; returns the base's positioning verdict unchanged. See the definition.
    bool split_adjust_position(size_t v_new, const std::vector<Tuple>& children) override;
    void split_after_vertex(size_t v_new, bool is_edge_open_boundary) override;

    /// The offset's surface can end on a non-manifold or boundary edge of the input complex,
    /// which the base must not flip or split across.
    bool is_open_boundary_edge(const Tuple& e) override { return is_order_2_edge(e); }

private:
    /// What the shared split has to carry across for the offset: the region tag of each parent
    /// tet, keyed by the edge opposite the split one, and which surfaces the edge was on.
    struct OptSplitCache
    {
        bool is_edge_on_input = false;
        bool is_edge_on_offset = false;
        std::map<simplex::Edge, TetAttributes> tets;
    };
    wmtk::threading::enumerable_thread_specific<OptSplitCache> m_opt_split_cache;

public:
private:
public:
private:
    /// The worst offset-face criterion around the operation, captured before it runs -- the bar
    /// its `after` hook compares against, mirroring how the AMIPS gates use cache.max_energy.
    wmtk::threading::enumerable_thread_specific<double> m_collapse_offset_rel_before;
    wmtk::threading::enumerable_thread_specific<double> m_swap_offset_rel_before;

    bool swap_capture_tag(const std::vector<size_t>& tids);
    /// The tag swap_after_cells writes onto the tets the swap created, chosen in `before`.
    wmtk::threading::enumerable_thread_specific<CellTag> m_swap_tag;
    /// The construction LABEL shared by every cell of the swap's ring, captured alongside the
    /// tag. Without this a swap leaves its new cells holding whatever label occupied the
    /// recycled tet slot, and the label is what region membership is read from.
    wmtk::threading::enumerable_thread_specific<int> m_swap_label;

public:
    /**
     * @brief initialize TetMesh from vertex, tet, and tag data
     * @param V: #V by 3 vertex matrix
     * @param T: #T by 4 tet matrix
     * @param T_tags: #T by #tags tag matrix
     * @param V_env: #V_env by 3 EnvelopeSurface vertices
     * @param F_env: #F_env by 3 EnvelopeSurface faces
     */
    void init_from_image(
        const MatrixXd& V,
        const MatrixXi& T,
        const MatrixSi& T_tags,
        const MatrixXd& V_env,
        const MatrixXi F_env,
        const std::vector<std::string>& tag_names);

    void init_surfaces_and_boundaries();

    /// Whether edge `loc` lies on the INPUT complex. The base's is_edge_on_input() asks
    /// about the union of the two tracked surfaces, and is_edge_on_bbox() is the base's.
    bool is_edge_on_input(const Tuple& loc);
    bool is_edge_on_offset(const Tuple& loc);

    /**
     * @brief The two simplex sets the optimization may not touch at all.
     *
     * The input simplicial complex is the geometry the offset is measured against, and the
     * domain boundary is the box the background mesh lives in. Neither is something to be
     * improved: an operation that splits, collapses, flips or moves any of their simplices
     * changes the reference the whole result is defined against. This is categorical, not a
     * tolerance -- the shared engine holds the input surface inside m_envelope, which lets it
     * drift by eps, and eps is not zero.
     *
     * Same rule and same wording as TopoOffsetTriMesh::vertex_is_frozen()/edge_is_frozen(). The
     * one difference the dimension forces: 2D reads an edge's bbox membership off the edge
     * itself, while 3D tracks the box on FACES, so the edge test goes through the base's
     * is_edge_on_bbox(), which walks the edge's incident faces. Both are therefore non-const.
     */
    /**
     * THE DOMAIN WALL IS NO LONGER PART OF THIS. It is a region boundary now, held in m_envelope
     * with every other one, and containment is the whole constraint on it -- the same contract
     * the input complex gets from the shared engine, which the paragraph above already describes
     * as a tolerance rather than a prohibition.
     *
     * Freezing it was the third leg of the degeneracy trap measured on prism (the other two,
     * both lifted: no split of a wall edge, no smoothing of a wall vertex). With collapse
     * refused as well, a wafer tet resting on the wall had NO legal repair at all -- every
     * remaining operation halved its volume while leaving its longest edge, which lies in the
     * wall, exactly as it was.
     *
     * The wall does not thereby become collapsible to nothing: TetOptimizerMeshCollapse still
     * requires a wall vertex to merge into one carrying at least as many bbox faces, so a corner
     * cannot dissolve into a face and the box keeps its shape.
     */
    bool vertex_is_frozen(const size_t vid) const { return m_vertex_extra[vid].m_is_on_input; }
    bool edge_is_frozen(const Tuple& loc) { return is_edge_on_input(loc); }

    /**
     * @brief check that the ambient tag does not overlap with any other tags
     */
    bool ambient_assert();

    /**
     * @brief label input simplicial complex simplices, as defined in
     * m_offset_params.offset_selection
     */
    void label_input_complex();

    /**
     * @brief check if the input complex is empty. Only valid after calling init_from_image(...).
     * Checks if any vertices (therefore any simplices) are labelled 1, if not returns true
     */
    bool empty_input_complex();

    /**
     * @brief Build the input complex's BVH and keep the extraction the potential needs.
     *
     * Must be called after init_from_image(...). One extraction feeds both, so the exact
     * distance field and the smooth potential can never describe different geometry.
     */
    void init_input_complex_bvh();

    /**
     * @brief Build the smooth offset potential from the extraction init_input_complex_bvh() kept.
     *
     * Separate from the BVH init because it needs target_distance and offset_dhat_factor, which
     * a caller that only wants the distance field has no reason to have set.
     */
    void init_offset_potential();

    /// The input complex as Phi's primitives: the BOUNDARY triangles of the label-1 tet region
    /// plus the complex's isolated triangles, every edge of those triangles plus the complex's
    /// wires, and its isolated points. Filled by init_input_complex_bvh(), consumed by
    /// init_offset_potential().
    MatrixXd m_phi_V;
    MatrixXi m_phi_E;
    MatrixXi m_phi_F;
    std::vector<int> m_phi_P;


    /**
     * @brief label connected simplicial complex components (simplices labelled 1 or 2)
     */
    size_t flood_fill();

    std::vector<std::array<size_t, 3>> get_faces_by_condition(
        std::function<bool(const FaceAttributes&)> cond) const;

    //// overriden splits/invariants
    /// Dispatch: the optimization phase runs the shared split, everything else the
    /// marching-tets one below.
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;
    bool marching_split_edge_before(const Tuple& t);
    bool marching_split_edge_after(const Tuple& t);
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
    bool split_tet_before(const Tuple& t) override;
    bool split_tet_after(const Tuple& t) override;
    bool invariants(const std::vector<Tuple>& tets) override;
    /// The base rounds and refuses bbox vertices; this drops the bbox refusal (the wall is a
    /// region boundary held in an envelope, not frozen) and, in Phase B, refuses every vertex
    /// that is not offset-only -- that phase places the offset surface and moves nothing else.
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    /**
     * @brief Phase B's placement of an offset-surface vertex: the offset energy ALONE, then
     * backtrack into the one-ring if the solved point inverts something.
     *
     * This is the ONLY smoothing Phase B does: smooth_before() refuses every other vertex in
     * that phase, so no AMIPS term is assembled anywhere in it.
     *
     * WHY THIS IS NOT THE SHARED SMOOTHER. There the objective is w_amips * AMIPS + the offset
     * term, and the vertex comes to rest where those two gradients cancel, which is not on the
     * level set: the header of SmoothVertexOptions says so outright ("the vertex rests a
     * w_amips-proportional distance off the input"). AMIPS is dimensionless, so its positional
     * gradient scales as 1/h -- refining the mesh makes the resting point WORSE, which is the
     * at-vertex wall this component kept hitting (measured on prism at 5%: the residual bottomed
     * at 1.75x and then climbed back to 2.05x as refinement continued, with ~380 faces carrying
     * an over-tolerance vertex).
     *
     * So the placement here is the offset condition only, and element shape is left entirely to
     * the split/collapse/swap passes, which is where this component already puts it (see
     * smooth_quality_veto, off by default for the same reason). It is a 1-D ROOT FIND along the
     * normal -- Phi(x) = c -- and NOT a 3-D minimisation of the offset energy: that energy's
     * Hessian is rank one, so minimising it leaves the level set's tangent plane unconstrained
     * and the vertex slides (measured with AMIPS removed and the 3-D solve kept: 19.31x
     * tolerance at round 1 against 8.6-9.0x with AMIPS, 1772 over-tolerance vertices against 0).
     * The one thing the placement must still respect is that no incident tet may invert, and
     * that is a constraint on the SEGMENT to the target rather than a reason to discard it:
     * bisect for the furthest point along it that keeps every tet valid.
     *
     * Nothing is lost by not calling the base for these vertices: smoothing_containment_envelope()
     * returns null for an offset-only vertex in Phase B, and the quality veto is already off.
     */
    bool smooth_offset_vertex_backtracking(const Tuple& t);
    //// overriden splits/invariants

    /**
     * @brief main function from which all others are called
     */
    void execute_offset(const std::filesystem::path& output_file);

    /**
     * @brief optimize the offset
     */
    void optimize_offset(const std::filesystem::path& output_file);

    //// smoothing

    /**
     * @brief true if face f has exactly one incident tet labelled 2 (offset), i.e. it lies on
     * the boundary between the offset region and the rest of the mesh
     */
    bool is_offset_face(const Tuple& f) const;
    bool is_offset_face(const size_t fid) const;

    /**
     * @brief offset-surface faces (see is_offset_face()) incident to vertex t
     */
    std::vector<Tuple> get_offset_surface_faces_for_vertex(const Tuple& t) const;

    /**
     * @brief Why smoothing refused a vertex, one counter per site where it can say no.
     *
     * The base's SmoothRejectCounters covers the rejections inside the shared smoother, which is
     * now every vertex; these cover the sites before it, where the offset says no for its own
     * reasons, plus the split of what reached the smoother into offset-surface and interior.
     */
    struct SmoothTrace
    {
        std::atomic<int> attempted{0}; ///< smooth_before() entered
        std::atomic<int> before_bbox{0}; ///< base smooth_before said no: on the bounding box
        std::atomic<int> before_unrounded{0}; ///< base smooth_before said no: could not round
        std::atomic<int> before_on_input{0}; ///< input complex, frozen
        std::atomic<int> before_phase_b_not_offset{0}; ///< Phase B: not an offset-only vertex
        std::atomic<int> offset_attempted{0}; ///< reached the smoother carrying the offset term
        std::atomic<int> interior_attempted{0}; ///< reached it as an ordinary interior vertex

        void reset()
        {
            for (std::atomic<int>* c :
                 {&attempted,
                  &before_bbox,
                  &before_unrounded,
                  &before_on_input,
                  &before_phase_b_not_offset,
                  &offset_attempted,
                  &interior_attempted}) {
                c->store(0);
            }
        }
    };
    mutable SmoothTrace m_smooth_trace;

    void log_smooth_trace() const;

    /**
     * @brief Which of four DISJOINT classes a vertex is accounted to when measuring movement.
     *
     * Bbox first, deliberately: a vertex may be on the bounding box AND on a tracked surface
     * (the offset region can reach the domain wall), and the bbox freeze in smooth_before()
     * overrides everything else, so for "does it actually move" it is the bbox that decides.
     * Offset before input for the same reason -- a vertex on both is a construction defect
     * check_no_vertex_on_both_surfaces() throws on, so the order there is academic.
     */
    enum class VClass { Bbox = 0, Offset = 1, Input = 2, Interior = 3, Count = 4 };

    VClass vertex_class(const size_t vid) const
    {
        if (!m_vertex_attribute[vid].on_bbox_faces.empty()) return VClass::Bbox;
        if (m_vertex_extra[vid].m_is_on_offset) return VClass::Offset;
        if (m_vertex_extra[vid].m_is_on_input) return VClass::Input;
        return VClass::Interior;
    }

    static const char* vclass_name(VClass c)
    {
        switch (c) {
        case VClass::Bbox: return "bbox";
        case VClass::Offset: return "offset";
        case VClass::Input: return "input";
        default: return "interior";
        }
    }

    /**
     * @brief How far each class of vertex actually gets during smoothing.
     *
     * Attempts and refusals alone cannot answer "is this surface making progress": a class can
     * be attempted constantly, accepted often, and still be going nowhere in tiny steps, which
     * reads identically to healthy movement in the accept counters. So displacement is summed
     * and maximised over ACCEPTED moves only -- the moves that survived every gate and left the
     * vertex somewhere new. A rejected move is either restored in place (the projected path) or
     * rolled back by the operation framework, so counting its displacement would measure an
     * intermediate the mesh never kept.
     *
     * Smoothing is the ONLY operation that moves an existing vertex: split inserts new ones,
     * collapse merges v1 into v2 at v2's position, and swaps only rewire. So `accepted == 0`
     * for a class is the same statement as "no vertex of this class ever moved".
     */
    struct MoveStats
    {
        std::atomic<int> attempted{0};
        std::atomic<int> refused_before{0};
        std::atomic<int> accepted{0};
        std::atomic<double> sum_disp{0.};
        std::atomic<double> max_disp{0.};

        void add(double d)
        {
            ++accepted;
            for (double cur = sum_disp.load(); !sum_disp.compare_exchange_weak(cur, cur + d);) {
            }
            for (double cur = max_disp.load();
                 d > cur && !max_disp.compare_exchange_weak(cur, d);) {
            }
        }
    };
    mutable std::array<MoveStats, size_t(VClass::Count)> m_move_stats;

    /**
     * @brief Which of two mechanisms lets a wall vertex end up off its wall.
     *
     * (1) STALE FLAGS: on_bbox_faces is inherited or kept by an operation for a vertex that is
     *     not on that wall -- collapse explicitly does not maintain it, which was safe only
     *     while wall vertices could not move.
     * (2) VACUOUS CONTAINMENT: the vertex really is on the wall and smoothing really does move
     *     it off, because the containment check found no tracked faces at it and so tested
     *     nothing.
     *
     * These are distinguished at the moment of an ACCEPTED smoothing move: record how many
     * tracked faces the vertex had, and how much off-plane deviation that one move introduced.
     * Moves that add deviation while the vertex had zero tracked faces are (2); an invariant
     * violation with no such moves at all is (1).
     */
    struct WallMoveStats
    {
        std::atomic<int> accepted{0}; ///< accepted smoothing moves on a vertex with wall flags
        std::atomic<int> accepted_zero_tracked{0}; ///< ... of those, with NO tracked face at it
        std::atomic<int> added_deviation{0}; ///< ... that increased off-plane deviation
        std::atomic<int> added_dev_zero_tracked{0}; ///< ... and had no tracked face: mechanism (2)
        std::atomic<double> max_single_step{0.}; ///< largest off-plane deviation one move added

        void note(bool zero_tracked, double dev_before, double dev_after)
        {
            ++accepted;
            if (zero_tracked) ++accepted_zero_tracked;
            const double d = dev_after - dev_before;
            if (d > 0.) {
                ++added_deviation;
                if (zero_tracked) ++added_dev_zero_tracked;
                for (double cur = max_single_step.load();
                     d > cur && !max_single_step.compare_exchange_weak(cur, d);) {
                }
            }
        }
    };
    mutable WallMoveStats m_wall_moves;

    /// How far vid sits from every bounding-box plane it claims membership of. Zero when it is
    /// where its on_bbox_faces say it is; empty membership gives zero.
    double wall_offplane_deviation(const size_t vid) const;

    /// Per-class smoothing movement, plus the direct bounding-box invariant check.
    void log_vertex_movement(const char* when) const;


    //// smoothing

    //// collapse

    /**
     * @brief Seed the sizing field from the offset surface's CURRENT edge lengths, once, before
     * the first pass. Paper Sec. 5.3.3 Step 1: "initialized with the current length of each edge."
     *
     * Without it the field starts at the base target (a fraction of the bounding box), which is
     * far coarser than the offset surface, so every offset edge is a collapse candidate on pass
     * one and the surface is decimated before any metric is computed.
     */
    void init_offset_sizing_field();

    /**
     * @brief How far the offset surface is from where it should be: {max, avg} over its vertices.
     *
     * The absolute error |dist(v, input complex) - target_distance|, over the band's OUTER
     * surface only -- a band cell meeting a plain-background cell, not the input complex it
     * wraps, whose interface sits at distance 0 by construction. The MAX is what the optimization
     * converges against: the offset is only as good as its worst-placed vertex.
     *
     * A face with no opposite tet is ON THE DOMAIN BOUNDARY and is counted, not skipped. The 2D
     * version skipped those and under-reported a bbox-clipped offset by 11x -- true error on the
     * clipped stretch reached 52% of the target distance while the report showed 0.5%. Those
     * vertices are frozen on the box and can never be fixed, which is exactly why they must be
     * visible rather than silently dropped.
     */
    std::pair<double, double> compute_distance_deviation() const;

    /**
     * @brief The band's outer surface, recomputed live rather than read from the cached class.
     *
     * A band cell meeting a cell that is neither band nor input complex. Live because the
     * operations that ask this run between one labelling pass and the next, so a face a split
     * just created carries whatever cached class its recycled slot happened to hold.
     *
     * Returns true for a band face on the domain boundary (no opposite tet) -- see
     * compute_distance_deviation() for why that case must not be dropped.
     */
    bool face_is_offset_surface_live(const Tuple& f) const;
    void diag_offset_bands(const char* tag) const;

    /// Whether tet `tid` is part of the offset BAND (as opposed to the input complex it wraps).
    bool cell_is_offset_band(const size_t tid) const { return m_tet_attribute[tid].label == 2; }

    /// Whether tet `tid` is part of the INPUT complex the band wraps.
    bool cell_is_input_complex(const size_t tid) const { return m_tet_attribute[tid].label == 1; }

    /**
     * @brief Warn if conservative growth ran into the bounding box.
     *
     * A band face on the domain boundary means target_distance exceeded the clearance between the
     * input complex and the box, so the offset is clipped there. Those vertices are frozen and no
     * operation can move them, which makes the target distance unreachable by construction --
     * worth saying out loud rather than leaving as a mysterious plateau in max_dist_err.
     */
    void warn_if_offset_reaches_domain_boundary() const;

    /// The vertex compute_distance_deviation() last found the max at, and a dump of everything
    /// that could be stopping it from moving. Diagnostic only.
    mutable size_t m_worst_dist_vid = static_cast<size_t>(-1);
    void log_worst_dist_vertex() const;

    /**
     * @brief DIAGNOSTIC: dump the single worst-quality tet and everything that could be pinning
     * it, so a Phase A that will not converge says WHICH element and WHY rather than only a max.
     *
     * Reports, for the max-cbrt(cell_quality) tet: its volume and inversion state, its region
     * label, and per vertex the flags that decide whether any operation may touch it -- on the
     * input complex, on the offset surface, on the bounding box, rounded, order, sizing scalar,
     * and vertex_is_frozen() (which is what collapse_before_vertex() refuses on). Then each of
     * the six edges against the two length gates, so it is visible whether split or collapse
     * would even consider them: split needs len^2 >= splitting_l2 * sbar^2, and the coarsening
     * pass stops at len^2 <= collapsing_l2 unless coarsen_unbounded.
     *
     * The point is to separate "no operation is being attempted here" from "operations are
     * attempted and refused" -- which the aggregate counters cannot distinguish.
     */
    void log_worst_tet(const char* when) const;

    /// {max_dist_err, avg_dist_err, max_norm_dev, avg_norm_dev} per optimization iteration.
    /// {max_dist_err, avg_dist_err, max_phi_residual, avg_phi_residual, max_grad, avg_grad}.
    /// Only the last pair is the convergence criterion; the first four are diagnostics kept
    /// because they answer different questions -- the Euclidean error says how far the smoothed
    /// offset ended up from the exact one, and the Phi residual still ranks the sizing field.
    std::vector<std::array<double, 8>> optimization_metrics;
    /// {split-born vertices, recollapsed, recollapsed in the immediately following collapse
    /// pass} per A/B round, in step with op_counts. See VertexExtra::m_born_epoch.
    std::vector<std::array<int, 3>> churn_counts;
    /// {splits, collapses, swaps} per A/B ROUND -- one entry per round the driver runs,
    /// including the round that converges, as deltas rather than running totals. Phase B does no
    /// topological work, so a round's entry is exactly what its Phase A did. NOTE this does NOT
    /// mirror optimization_metrics, which is still a single whole-run summary.
    std::vector<std::array<int, 3>> op_counts;
    /// Whether the optimization met both convergence criteria before the iteration cap.
    bool m_converged = false;

    /// Per-iteration operation counters, reset before each iteration's operation passes.
    /// CHURN: split-born vertices that a collapse later removed, and the subset removed in the
    /// same pass-pair that created them (born in split pass N, gone in the collapse pass that
    /// immediately follows it). The rest survived at least into a later iteration.
    std::atomic<int> iter_cnt_split_born{0};
    std::atomic<int> iter_cnt_recollapsed{0};
    std::atomic<int> iter_cnt_recollapsed_same_pass{0};
    std::atomic<int> iter_cnt_split{0};
    std::atomic<int> iter_cnt_collapse{0};
    std::atomic<int> iter_cnt_swap{0};
    std::atomic<int> iter_cnt_collapse_offset_removed{0};
    /// Operations refused because they would have left an offset face over tolerance.
    std::atomic<int> iter_cnt_collapse_offset_reject{0};
    std::atomic<int> iter_cnt_swap_offset_reject{0};
    /// Splits of an OFFSET-surface edge: reached split_edge_after, accepted, refused there.
    std::atomic<int> iter_cnt_split_offset_before{0};
    std::atomic<int> iter_cnt_split_offset_endpoints{0};
    std::atomic<int> iter_cnt_split_offset_frozen{0};
    std::atomic<int> iter_cnt_split_offset_base_reject{0};
    std::atomic<int> iter_cnt_split_offset_tried{0};
    std::atomic<int> iter_cnt_split_offset{0};
    std::atomic<int> iter_cnt_split_offset_reject{0};


    //// collapse


    //// sizing field
    // Bounds and thresholds for the sizing field are user-configurable; see
    // Parameters::min_sizing_scalar / max_sizing_scalar / sizing_mrm_threshold /
    // sizing_gradation. Refinement stops at min_sizing_scalar; coarsening never exceeds
    // max_sizing_scalar (default: the base target itself, since a sizing_scalar > 1 would mean
    // "coarser than what the user asked for", which update_sizing_field() never has a reason
    // to produce).

    /**
     * @brief unsigned mean ratio metric of a triangle: 2*sqrt(3)*area / (sum of squared edge
     * lengths). 1 for equilateral, -> 0 as the triangle degenerates. See
     * https://github.com/wildmeshing/topological-offsets/blob/main/components/topological_offsets/wmtk/components/topological_offsets/internal/utils/mean_ratio_metric.hpp
     */
    static double mean_ratio_metric(const Vector3d& p0, const Vector3d& p1, const Vector3d& p2);

    /**
     * @brief refine or coarsen the sizing field (VertexAttributes::m_sizing_scalar) based on
     * the mean ratio metric of the offset triangulation (offset-class surface faces), following
     * the reference's compute_target_edge_length(): for every vertex incident to at least one
     * offset-surface face, take the worst (minimum) mean_ratio_metric() among those faces, and
     * halve the sizing scalar if it is below m_offset_params.sizing_mrm_threshold, or multiply it
     * by 1.5 (coarsen) if above -- clamped to [m_offset_params.min_sizing_scalar,
     * m_offset_params.max_sizing_scalar]. Vertices not incident to any offset-surface face are left
     * untouched. The vertices actually refined this pass then seed
     * wmtk::utils::gradation_smooth_sizing() (m_offset_params.sizing_gradation), matching
     * SimWildMesh::gradation_smooth_sizing(), so a newly refined patch doesn't sit right next
     * to an unrelated coarse one. Called once per optimize_offset() iteration, after smoothing.
     * @note skeleton: unlike the reference, this doesn't also factor in normal deviation.
     */
    //// sizing field

    //// swap

    /**
     * @brief execute simplistic marching tets. All edges with one vertex labelled 0 and the other
     * 1/2 are split, at the MIDPOINT (Midpoint) or at a fraction of the way toward the input
     * complex (Initial).
     *
     * The distance-field modes that used to exist here -- BinarySearch, LogRootFind and
     * SphereTracing, which root-found each band-boundary edge onto d(x) = delta at insertion time
     * -- are gone, as they are in 2D. They contradict the paper, which places inserted vertices at
     * the midpoint (Sec. 5.2: "we do not perform an interpolation ... we just place the inserted
     * vertices at the midpoint of the split edges") and leaves the distance entirely to the
     * Step-3 optimization; and they did the optimizer's job with a worse tool, having no error
     * feedback and running none of the accept checks the shared smoother does.
     */
    void marching_tets();


    //// simplicial embedding stuff
    /**
     * @brief check if the input complex (simplices labelled 1) are simplicially embedded w.r.t. the
     * entire mesh
     */
    bool is_simplicially_embedded() const;

    /**
     * @brief check if a tet satisfies simpicial embedding criteria w.r.t. input complex
     * (simplices labelled 1)
     */
    bool tet_is_simp_emb(const Tuple& t) const;

    /**
     * @brief make mesh a simplicial embedding of the input complex (simplices labelled 1)
     */
    void simplicial_embedding();
    //// simplicial embedding stuff

    //// variable offset stuff
    /**
     * @brief check if removing the tet would change the topology of any label
     */
    bool tag_tet_consistent_topology(size_t t_id, int64_t tag) const;

    /**
     * @brief check if adding a tet to the offset region does not change the topology of the
     * offset. Returns true if topology would not be changed
     */
    bool offset_tet_consistent_topology(const size_t t_id) const;

    /**
     * @brief check if a tet is inside the offset (implicitly defined via BVH distance field to
     * input complex) via conservative sphere subdivision estimation
     */
    bool tet_is_in_offset_conservative(const size_t t_id, const double threshold_r) const;

    /**
     * @brief check if a tet is inside the offset (implicitly defined via BVH distance field to
     * input complex) by check if all its vertices are inside the offset region.
     */
    bool tet_is_in_offset_aggressive(const size_t t_id) const;

    /**
     * @brief grow offset region conservatively using conservative checks while ensuring consistent
     * topology
     */
    void grow_offset_conservative();

    /**
     * @brief Grow offset region aggressively. A tet is considered in the offset if all its vertices
     * are in the offset.
     */
    void grow_offset_aggressive();
    //// variable offset stuff

    /**
     * @brief update 'tags' data for tets in the offset region (tets labelled 2) based on
     * the given offset tag values in m_offset_params.offset_tag_value
     */
    void set_offset_tet_tags();

    /**
     * @brief verify that the closed offset region (simplices labelled 1 or 2) form a manifold
     * region. This should be true for any offset. This function is for verification.
     * @note We first collect the tets labelled 1 or 2, then extract the boundary of this region
     * and check if it is manifold.
     */
    bool offset_is_manifold();

    //// output stuff
    void write_input_complex(const std::string& path); // write components labeled to be offset
    void write_vtu(const std::string& path);
    // void write_msh(const std::string& file);
    void write_msh_groups(const std::string& file);
    //// output stuff

private:
    /**
     * @note for all split caches, simplex attributes are inherited from the simplex (of same or
     * higher order) they are 'borne' out of
     */

    struct EdgeSplitCache
    {
        size_t v1_id;
        size_t v2_id;
        // The marching-tets placement and the offset's own data for the new vertex. The
        // shared attributes (position, rounding, bbox, order) are written in `after`.
        Vector3d new_v_pos;
        VertexExtra new_v_extra;

        bool is_edge_on_input = false;
        bool is_edge_on_offset = false;
        bool is_edge_open_boundary = false;

        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;

        // cache edge attributes
        EdgeAttributes split_e;
        std::map<size_t, EdgeAttributes> internal_e;
        std::map<simplex::Edge, EdgeAttributes> external_e; // edge is boundary edge (not link)
        std::map<simplex::Edge, EdgeAttributes> link_e; // link edge around splitted edge

        // cache face attributes
        std::map<size_t, FaceSnapshot> split_f; // splitted faces
        std::map<simplex::Edge, FaceSnapshot> internal_f; // new faces created by split
        std::map<std::pair<simplex::Edge, size_t>, FaceSnapshot>
            external_f; // closed star boundary faces of splitted edge

        // cache tet attributes
        std::map<simplex::Edge, TetAttributes> tets;
    };
    wmtk::threading::enumerable_thread_specific<EdgeSplitCache> edge_split_cache;

    struct FaceSplitCache
    {
        size_t v1_id;
        size_t v2_id;
        size_t v3_id;

        // cache edge attributes
        std::map<simplex::Edge, EdgeAttributes> existing_e;

        // cache face attributes
        std::map<simplex::Face, FaceSnapshot> existing_f;
        int splitf_label;

        // cache tet attributes
        std::map<size_t, TetAttributes> tets;
    };
    wmtk::threading::enumerable_thread_specific<FaceSplitCache> face_split_cache;

    struct TetSplitCache
    {
        std::array<size_t, 4> v_ids;

        // cache retained edge attributes
        std::map<simplex::Edge, EdgeAttributes> existing_e;

        // cache retained face attributes
        std::map<simplex::Face, FaceSnapshot> existing_f;

        // cache tet attribute
        TetAttributes tet;
    };
    wmtk::threading::enumerable_thread_specific<TetSplitCache> tet_split_cache;


public:
    // substructure functions

    bool is_order_2_edge(const Tuple& e) const;
    bool is_order_2_edge(const std::array<size_t, 2>& e) const;

    bool vertex_is_on_surface(const size_t vid) const override;

    bool face_is_on_surface(const size_t fid) const override;

    size_t get_order_of_vertex(const size_t vid) const override;
    /**
     * @brief Compute the vertex order for every vertex.
     */
    void init_vertex_order();

private: // helpers
    /**
     * @brief determine if any tag from tag1 is also present in tag2.
     * @note if tag2 is empty (ambient), return true if tag1 is empty, otherwise false (tag2 is
     * ambient, so only 'element' is ambient)
     */
    bool any_tag_present(const CellTag& tag1, const CellTag& tag2)
    {
        if (tag2.empty()) {
            return tag1.empty();
        }
        if (tag1.empty()) { // tag1 is ambient, tag2 is not
            return false;
        }

        for (const int64_t& i : tag1) {
            if (tag2.find(i) != tag2.end()) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief sort edge simplices in place by decreasing edge length
     */
    void sort_edges_by_length(std::vector<simplex::Edge>& edges)
    {
        std::sort(
            edges.begin(),
            edges.end(),
            [this](const simplex::Edge& e1, const simplex::Edge& e2) {
                double len1 = (m_vertex_attribute[e1.vertices()[0]].m_posf -
                               m_vertex_attribute[e1.vertices()[1]].m_posf)
                                  .norm();
                double len2 = (m_vertex_attribute[e2.vertices()[0]].m_posf -
                               m_vertex_attribute[e2.vertices()[1]].m_posf)
                                  .norm();
                return len1 > len2;
            });
    }

public: // helpers
    /**
     * @brief assign each vertex a partition id (by spatial Morton order) for the parallel
     * ExecutePass policies (kPartition/kColor) used by smooth_all_vertices()/
     * collapse_all_edges(). A no-op if NUM_THREADS == 0.
     */
    void compute_vertex_partition();

    size_t get_partition_id(const Tuple& loc) const
    {
        return m_vertex_attribute[loc.vid(*this)].partition_id;
    }

    /**
     * @brief get tets (as Tuples) that are face-adjacent to the given tet (as Tuple)
     */
    std::vector<Tuple> get_face_adjacent_tets(const Tuple& t) const
    {
        std::vector<Tuple> adj_tets;
        auto tet_1 = t.switch_tetrahedron(*this);
        if (tet_1) {
            adj_tets.push_back(tet_1.value());
        }
        auto tet_2 = t.switch_face(*this).switch_tetrahedron(*this);
        if (tet_2) {
            adj_tets.push_back(tet_2.value());
        }
        auto tet_3 = t.switch_edge(*this).switch_face(*this).switch_tetrahedron(*this);
        if (tet_3) {
            adj_tets.push_back(tet_3.value());
        }
        auto tet_4 =
            t.switch_vertex(*this).switch_edge(*this).switch_face(*this).switch_tetrahedron(*this);
        if (tet_4) {
            adj_tets.push_back(tet_4.value());
        }
        return adj_tets;
    }

    /**
     * @brief get all one-ring vertices through input simplices (labelled 1)
     */
    std::vector<size_t> connected_components_helper(const size_t& v_id)
    {
        auto onering_v_ids = get_one_ring_vids_for_vertex(v_id);
        std::vector<size_t> ret_v_ids;
        for (const size_t& other_v_id : onering_v_ids) {
            size_t e_id = tuple_from_edge({{v_id, other_v_id}}).eid(*this);
            if (m_edge_attribute[e_id].label != 0) { // edge labelled 1 or 2
                ret_v_ids.push_back(other_v_id);
            }
        }
        return ret_v_ids;
    }

    /**
     * @brief reset connected component assignments.
     */
    void reset_connected_components()
    {
        auto verts = get_vertices();
        for (const Tuple& v : verts) {
            size_t v_id = v.vid(*this);
            m_vertex_extra[v_id].component_id = 0;
        }
    }
};


} // namespace wmtk::components::topological_offset
