#pragma once

#include <array>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <mutex>
#include <string>

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

const int64_t TEMP_OFFSET_TET_TAG = -1;
const CellTag TEMP_OFFSET_TET_TAG_SET{TEMP_OFFSET_TET_TAG};


// for all attributes:
// label: 0=default, 1=input, 2=offset

/**
 * @brief Per-vertex data the shared optimizer knows nothing about.
 *
 * Position, rounding, bbox, order, sizing and partition live on the base's VertexAttributes;
 * this carries only the offset's own, registered with m_vertex_attr_group so it is resized,
 * protected and rolled back with them. The offset tracks two surfaces: the base's
 * m_is_on_surface is their union; m_is_on_region / m_is_on_offset say which.
 */
class VertexExtra
{
public:
    int label = 0;
    size_t component_id = 0;
    /// On a region boundary that is not the offset surface. The whole family -- this flag,
    /// is_edge_on_region(), face_is_region() -- means "region boundary"; 3D has no
    /// INPUT_SURFACE_CLASS to narrow it with the way 2D does.
    bool m_is_on_region = false;
    bool m_is_on_offset = false; // is this vertex on the offset surface

    /// On the input complex itself -- the simplices label_input_complex() selected, at every
    /// dimension. A vertex is never on the input complex and the offset surface at once;
    /// check_no_vertex_on_both_surfaces() asserts it, and collapse_before_vertex() refuses the
    /// merge that could manufacture it. Propagated like the flags above: a split midpoint ANDs
    /// its edge's endpoints, a collapse ORs onto the survivor.
    bool m_is_on_input_complex = false;

    /**
     * Which tag-region boundaries this vertex lies on, one bit per input tag (m_tag_bit).
     *
     * Seeded at init from the boundary faces; a split midpoint ANDs its edge's endpoints, a
     * collapse survivor ORs in the removed vertex's bits. A face's mask is the AND of its three
     * corners', which is what envelope_for_mask() dispatches on. Never recomputed from current
     * tet tags, so the band's tag rewriting during construction cannot corrupt it.
     */
    uint64_t m_boundary_mask = 0;

    /**
     * Which split pass created this vertex, 0 if no optimization split did. Churn
     * instrumentation only; nothing reads it to make a decision. Assigned, never OR'd or left
     * alone: v_id may be a recycled slot still carrying a dead vertex's epoch. Zero also covers
     * the construction-era vertices, which predate every split pass.
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
 * Tracked-surface state lives on wmtk::SurfaceTagAttributes; only the construction label is the
 * offset's own, and only the pre-optimization phase reads it. Registered with m_face_attr_group
 * so it survives the mesh changing shape, but its values are maintained only by the offset's own
 * operations -- sound precisely because the label is dead by the time the shared ones run.
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
 * Construction -- marching tets, simplicial embedding, growing the offset region -- uses
 * TetMesh's splits directly; the optimization after it is wmtk::TetOptimizerMesh's, with the
 * offset supplying only policy through the hooks. Two surfaces are tracked: the input complex
 * (class 0, envelope-held) and the offset boundary (OFFSET_SURFACE_CLASS, free to move, defined
 * by tet labels rather than input geometry). The base's m_is_on_surface is their union, and that
 * union is what an operation must not tear.
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
     * The two criteria are optimized in turn, never jointly: driven from one loop they compete,
     * split refining the band and collapse removing on quality grounds what it made.
     *
     * Phase A is TetWild and nothing else -- same operations, gates, sizing field and
     * stall-driven refinement, with no offset energy term, acceptance criterion or stop metric.
     * Its one addition is m_offset_envelope, holding the offset surface within one Phi tolerance
     * of wherever Phase B last placed it, so "do not degrade the offset" is a geometric
     * constraint every operation already honours rather than a per-operation criterion.
     *
     * Phase B moves the offset surface and nothing else: smoothing against the offset energy to
     * a fixed point, no envelope on the offset (it is what has to travel), no topological
     * operations. Both phases write the shared sizing field -- A on element quality, B on the
     * Phi residual of faces smoothing could not place.
     */
    enum class OptPhase { A, B };

    /// Which phase is running. Read by every hook that differs between them; see OptPhase.
    OptPhase m_phase = OptPhase::A;

    /**
     * @brief Phase B's two sub-sweeps, run in this order within every smoothing pass.
     *
     * Offset: place every offset-surface vertex on the level set, by the 1-D minimization of
     * (Phi - c)^2 along grad Phi, backtracked into the one-ring if the minimum lies outside it.
     * Background: relax the interior by AMIPS, under the surface those placements just defined.
     *
     * The sweeps must stay separate and in this order, so a pass means "place, then relieve what
     * the placement cost". Interleaving would make each vertex's one-ring a moving target for its
     * neighbours, so the backtracking a placement hits would depend on visit order rather than on
     * geometry. smooth_before() reads this and refuses the other class outright, so the two
     * sweeps share the executor, counters and accept gates and differ only in who they admit.
     */
    enum class PhaseBSub { Offset, Background };
    PhaseBSub m_phase_b_sub = PhaseBSub::Offset;

    /**
     * @brief The tube the offset surface may not leave during Phase A. Null in Phase B.
     *
     * Rebuilt at the start of every Phase A from the offset surface as Phase B just left it,
     * with eps = offset_envelope_rel x the Phi tolerance. Rebuilding is what lets the surface
     * travel across rounds: each Phase A pins it near its current position, each Phase B is free
     * to move it. m_envelope, the input complex's, is built once and never rebuilt -- that
     * geometry is what the offset distance is measured against and may not drift at all.
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
    /// max_rounds is reached. Replaces the single mesh_improvement() call.
    void optimize_offset_alternating();

    /// Phase B's smoothing loop. Each pass sweeps every vertex once: offset vertices are placed
    /// on the level set by smooth_offset_vertex_backtracking(), background vertices minimize
    /// their one-ring AMIPS by smooth_interior_vertex_phase_b(). Returns the number of passes;
    /// the natural exit is a pass in which no offset vertex was backtracked by its one-ring
    /// (m_phase_b_constrained == 0), i.e. every local minimum is interior. See the definition
    /// for the other exits.
    size_t phase_b_smooth();

    /**
     * @brief Phase B's placement of a background vertex: Newton on the one-ring AMIPS energy,
     * to this vertex's own minimum.
     *
     * The counterpart of smooth_offset_vertex_backtracking() for vertices that carry no surface.
     * Solving the interior alongside the surface opens one-rings that would otherwise force the
     * offset root find to backtrack -- which is the pass loop's exit criterion.
     *
     * Reuses the shared smoother (optimization::smooth_vertex_3d) -- pure AMIPS, no envelope
     * terms, exact inversion accept, quality veto -- but with Phase B's own solver, which stops
     * at vertex_grad_tol_rel of the visit's initial gradient instead of a fixed iteration budget.
     */
    bool smooth_interior_vertex_phase_b(const Tuple& t);

    /**
     * @brief L-inf over offset vertices of the gradient of each vertex's own placement objective
     * -- the energy Phase B's Newton solves minimize. Exactly 0 at the Gauss-Seidel fixed point
     * whatever the residual, so it separates "placement finished" from "placement blocked": a
     * refused move contributes zero displacement but full gradient. Vertices with a
     * float-inverted or unrounded incident tet are skipped, as the smoother skips them.
     */
    double phase_b_band_gradient_linf();

    /**
     * @brief Per-vertex band sizing update, run after Phase B. Returns the number changed.
     *
     * Refine-only, and only on pure chord error. "In tolerance" is the convergence criterion
     * itself -- |grad (Phi - c)^2| <= offset_gradient_tolerance() -- measured at band vertices
     * and at face-interior samples on the lattice for_each_offset_face_sample() defines, so this
     * responds to the quantity that decides the run rather than to a proxy for it.
     *
     *  - Halve when the vertex and every surface one-ring neighbour are in tolerance but some
     *    sample inside an incident surface face is not: pure resolution error, the one thing a
     *    finer sizing field can fix.
     *  - Otherwise leave it alone. A vertex out of tolerance is misplaced, not under-resolved,
     *    and a vertex in tolerance with an out-of-tolerance neighbour is that neighbour's problem.
     *
     * This is the only thing that refines the sizing field for the offset. Phase A keeps
     * TetWild's quality-driven stall response (refine_sizing_around_worst), which ranks elements
     * by AMIPS; the two write the same field and accumulate.
     *
     * Clamped below by max(min_sizing_scalar, min_edge_length / l) -- the band's floor, not
     * stuck_refine_min_scalar -- and the changed vertices seed gradation_smooth_sizing(), so a
     * re-sized patch does not sit against an untouched one.
     */
    size_t update_band_sizing_from_tolerance();

public: // mode for splitting in marching tets
    enum class EdgeSplitMode {
        Midpoint = 0, // construction: simplicial embedding AND marching_tets
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
     * The offset surface is the level set Phi = c. Built from the same extraction as
     * m_input_complex_bvh, in the same call, so the two describe the same geometry, and like the
     * BVH built once and never rebuilt: every offset quantity is measured against the input as
     * given, not against whatever the mesh has drifted to.
     */
    std::shared_ptr<OffsetPotential3D> m_offset_potential;

    /**
     * @brief The exact-kind envelope of the input complex, built only for offset_field
     * "euclidean". Null otherwise.
     *
     * Not a containment envelope and never used as one: no operation tests against it. It exists
     * because nearest_point_feature() -- the foot point plus the feature kind the exact distance
     * derivatives case on -- is only answered by the exact path. Built from the same extraction
     * as m_input_complex_bvh, so it describes the same geometry.
     */
    std::shared_ptr<SampleEnvelope> m_input_complex_envelope;

    /**
     * @brief One containment envelope per input tag, ambient included. Both phases.
     *
     * E_t is a tube of half-width m_envelope_eps around the boundary faces of region t as the
     * input mesh carried them, built in init_surfaces_and_boundaries() before offset
     * construction, so the band's later tag rewriting never enters them. A simplex on several
     * boundaries is constrained by the intersection of its tags' tubes (envelope_for_mask()),
     * which pins junction curves and points to the junction itself. Every simplex of the input
     * complex lies on tag boundaries (see label_input_complex()), so these tubes hold all of it.
     *
     * m_envelope (the base's pointer) is a UnionEnvelope over these members, so the shared
     * engine's one direct use -- the collapse_edge_before point check -- keeps union semantics.
     *
     * Interior vertices of the complex are not held by these: a face interior to a region has
     * identical tag sets on both sides and lands in no bucket, so a tet-filled complex's
     * interior is free to optimise.
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
     * The offset works in doubles throughout, so every vertex it places is rounded and its
     * rational position is the exact value of the double it carries. m_pos must still be filled:
     * the shared split falls back to the exact midpoint when the double one would invert a tet,
     * and every quality and orientation test around an unrounded vertex reads its neighbours'
     * m_pos. A stale m_pos would silently feed those the wrong point.
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
     * The 3D counterpart of TopoOffsetTriMesh::face_in_region. The region is the offset band
     * plus the input complex it wraps, both named by tags, and tags are what the shared
     * operations propagate. Must not be read off the construction label instead: that is derived
     * state, stale the moment a split or swap creates a cell it was never written for.
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
    bool face_is_region(const size_t fid) const
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

        // TetWild parity, forced on for 3D regardless of the config key: Phase B places the
        // offset surface with its own root find, which never consults this flag, and Phase A is
        // pure TetWild quality work, where the veto is on. The spec default is false for the 2D
        // pipeline, whose offset-boundary placement the veto would freeze.
        _m_offset_params.smooth_quality_veto = true;

        optimization::deactivate_opt_logger();
    }

    ~TopoOffsetTetMesh() override = default;

    ////// wmtk::TetOptimizerMesh hooks

    double cell_quality(const size_t tid) const override { return m_tet_attribute[tid].m_quality; }
    void set_cell_quality(const size_t tid, const double q) override
    {
        // Spike tracker. Every operation that writes a quality comes through here, so this is
        // the one place that sees a tet cross from ordinary into absurd, and the spike's
        // position in the log names the pass that caused it. Only the crossing is reported, and
        // only the first few in full, or the flood would be the whole log.
        if (m_spike_track) {
            const double before = m_tet_attribute[tid].m_quality;
            const double a = spike_amips(q), b = spike_amips(before);
            // Two events, not a fixed bar -- a fixed bar spends the whole dump budget on the
            // mildest tets and never reaches the extremes.
            //   genesis: an ordinary tet ruined by one operation, i.e. the causal event.
            //   record:  a new global maximum, so the dumps end at the true worst and, since
            //            records only increase, the output stays bounded.
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
     * Prints the four vertices with the two things that separate the plausible causes -- their
     * rounding state (an unrounded vertex is exact-rational, and its float image can make a valid
     * tet look degenerate, which is why is_inverted_f and is_inverted are separate predicates)
     * and the float volume against the edge lengths, a genuine sliver against a coordinate
     * blow-up.
     */
    void log_quality_spike(size_t tid, double before, double after, const char* kind) const;

    /// Diagnostic: whether set_cell_quality reports tets crossing into absurd quality.
    bool m_spike_track = true;
    /// AMIPS above which a tet counts as absurd rather than merely bad.
    double m_spike_threshold = 1e5;
    /// AMIPS below which a tet counts as ordinary, so ordinary -> absurd is one operation's doing.
    double m_spike_ordinary = 100.;
    /// Largest AMIPS seen so far; a new maximum is logged, which bounds the output.
    mutable double m_spike_record = 0.;
    mutable std::atomic<int> m_spike_dumps{0};
    mutable std::atomic<int> m_spike_genesis_dumps{0};
    mutable std::atomic<int> m_spike_count{0};
    mutable std::atomic<int> m_spike_genesis{0};
    int m_spike_dump_budget = 24;

    /**
     * @brief Is this vertex on a region boundary -- a tag boundary, or the domain wall.
     *
     * Derived, never stored. Both halves are already maintained exactly: m_is_on_region by
     * is_edge_on_region() at each split and an OR at each collapse, on_bbox_faces by the
     * intersection of the split endpoints and by the collapse rule that a wall vertex may only
     * merge into one at least as constrained. A stored flag would be a fourth thing to keep in
     * step with those, carrying nothing they do not.
     */
    bool vertex_is_on_region(const size_t vid) const
    {
        return m_vertex_extra[vid].m_is_on_region || !m_vertex_attribute[vid].on_bbox_faces.empty();
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
     * @brief The tag boundaries this vertex lies on -- the raw mask, gated on the vertex still
     * being region geometry at all.
     *
     * The gate is required, not redundant. m_boundary_mask propagates by a bare AND of a split's
     * endpoints, so an edge whose two ends share a bit hands that bit to its midpoint even when
     * the edge is a chord through the interior -- and the front is built by splitting precisely
     * such edges. vertex_is_on_region() does not over-claim that way, so the mask says which
     * boundaries and the gate says whether the vertex is on one at all; the mask only ever
     * narrows an answer the flags already allow.
     */
    uint64_t vertex_boundary_mask(const size_t vid) const
    {
        return vertex_is_on_region(vid) ? m_vertex_extra[vid].m_boundary_mask : uint64_t(0);
    }

    /// A face lies on a boundary only if all of it does: the AND of its corners' masks.
    uint64_t face_mask(const std::array<size_t, 3>& vids) const
    {
        return vertex_boundary_mask(vids[0]) & vertex_boundary_mask(vids[1]) &
               vertex_boundary_mask(vids[2]);
    }

    /**
     * @brief The envelope a simplex with this boundary mask is contained in, or null.
     *
     * Zero bits: no boundary, no container. One bit: that tag's own envelope. Several bits: a
     * memoized IntersectionEnvelope over the members -- inside means inside every tube, which
     * pins junction geometry to the junction. The multi-bit composite is containment-only: it
     * implements just the virtual is_outside queries, so it must never be returned from
     * smoothing_energy_envelope(), whose caller uses the non-virtual nearest_point.
     */
    std::shared_ptr<SampleEnvelope> envelope_for_mask(uint64_t mask) const;

    std::shared_ptr<SampleEnvelope> surface_envelope_for_face(
        const std::array<size_t, 3>& vids) const override
    {
        // Boundary geometry first, in both phases: a face on any tag-region boundary may not
        // drift out of that boundary's tube, and a face on several is held in their
        // intersection. The mask carries the input complex too -- every complex simplex lies on
        // tag boundaries -- and E_t is built from the input mesh before construction touches it.
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
    /// surfaces need it: the offset boundary is re-triangulated constantly, and refusing its
    /// diagonals leaves badly-shaped triangles the sizing field then chases.
    bool allow_surface_swap() const override { return true; }
    bool check_surface_topology() const override { return m_offset_params.perform_sanity_checks; }

    /**
     * @brief The offset surface is the one tracked surface with NO envelope, in either role.
     *
     * It is the surface the optimization exists to move: a tube around wherever conservative
     * growth left it would cap how far it can travel toward the level set. What holds it is the
     * offset term in the objective, not a container.
     *
     * The pull must be a real envelope, never a composite: this hook's consumers call the
     * non-virtual SampleEnvelope queries (nearest_point, and the ExactDistanceEnergy3D trio),
     * which on a composite bind to the base's null BVH. So a junction vertex is pulled toward
     * its most-violated member tube instead -- one real envelope per smoothing attempt,
     * alternating toward the junction across passes -- while smoothing_containment_envelope()
     * enforces the full constraint.
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
     * set on, which here covers both tracked surfaces. m_envelope is built from the input
     * complex's boundary, so answering it for an offset-surface vertex would demand the offset
     * stay within eps of the input -- a tube of the wrong radius around the wrong surface, which
     * refuses every move the offset term asks for.
     *
     * The input complex falls through to the base and is contained by m_envelope in every phase.
     * That is the whole constraint on it: it is smoothed like any other TetWild surface vertex
     * and this tube is the tolerance it may drift within.
     *
     * The offset surface is the exception only in Phase A, where m_offset_envelope contains it
     * like any other tracked surface: Phase A minimises AMIPS alone, so without a container
     * nothing stops it relocating the offset surface for element shape, which Phase B would then
     * have to undo. Phase B answers null, being the pass whose job is to move the surface.
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

    // Do not re-add a smoothing_extra_energy() override; measured worse -- see git history of
    // this file. Phase B places the offset surface with its own root find and Phase A holds it
    // in m_offset_envelope, so the base's nullptr is correct in both phases.

    /// The driver hands a bare `debug_N`; put the frames beside the run's own output rather than
    /// in whatever directory it was launched from, which is what lets the viewer find them as a
    /// timeline -- it globs `*debug_*.vtu` in the output directory and orders them by counter.
    void write_optimization_debug_output(const std::string& path) override
    {
        // The per-pass debug_{N} series is opt-in: the driver's phase_{round}{A|B} frames are
        // the timeline the viewer shows, and the per-pass frames cost more time and disk than
        // the rest of the run put together.
        if (!m_offset_params.debug_output_per_pass && path.rfind("debug_", 0) == 0) {
            return;
        }
        write_vtu(m_offset_params.output_path + "_" + path);
    }

    /**
     * @brief Phase A's stall-driven sizing refinement -- TetWild's, on element quality.
     *
     * mesh_improvement() fires this when optimization_quality_stats()'s max stalls, and only
     * runs inside Phase A, so this only ever runs there; it ratchets the sizing down around the
     * worst tets by AMIPS, as TetWildMesh does. It does not refine for the offset -- that is
     * update_band_sizing_from_tolerance(). Both write the same per-vertex scalar.
     */
    size_t refine_sizing_around_worst(double) override;

    // optimization_bare_coarsen_passes() is deliberately not overridden: TetWild's opening and
    // closing unlimited-length collapse passes apply. Phase A rebuilds m_offset_envelope before
    // every mesh_improvement(), so the offset surface is held during them exactly as TetWild's
    // input surface is, and decimating to what the envelope tolerates is the contract.

    /**
     * @brief (max, avg) Phi residual over the reachable offset surface, in units of its
     * tolerance; the engine's stop metric is therefore 1.0.
     */
    std::tuple<double, double> optimization_quality_stats() override;

    /// Phase A is TetWild, so it stops where TetWild stops: absolute AMIPS against stop_energy,
    /// the base's default. Phase B's metric is the Phi residual normalized by its own tolerance,
    /// so 1.0 means converged. Both must agree with optimization_quality_stats() above: a metric
    /// in one unit tested against a bar in another stalls the phase.
    double optimization_stop_metric() const override
    {
        return m_phase == OptPhase::A ? wmtk::TetOptimizerMesh::optimization_stop_metric() : 1.;
    }

    /// The residual scale, derived from the criterion rather than configured beside it: grad E =
    /// 2 (Phi - c) grad Phi, so on a unit-slope field the bound |grad E| <= g is |Phi - c| <= g/2.
    /// It must stay derived -- it also sizes the Phase A offset envelope (offset_envelope_rel x
    /// this) and the min_edge_length floor, and as its own knob it would not follow
    /// front_conv_rel, holding the offset surface far tighter than the run is judged by.
    double offset_residual_tolerance() const
    {
        return std::max(
            0.5 * m_offset_params.front_conv_rel * m_offset_params.target_distance,
            1e-16);
    }

    /**
     * @brief The convergence tolerance: the bound on |grad (Phi - c)^2| at a band vertex.
     *
     * A fraction of target_distance, the right unit: grad E = 2 (Phi - c) grad Phi, and grad Phi
     * is dimensionless for a field whose value is a length, so grad E is a length. Stated on the
     * gradient rather than the residual because it needs no conversion from field value to
     * length: it is the stationarity condition of the objective Phase B minimises, so it is the
     * same test for every potential, which lets a concave input be judged by the same number as
     * a convex one. Where |grad Phi| departs from 1 -- a medial axis, a sharp feature, competing
     * primitives -- the two bounds are different tests and neither implies the other.
     */
    double offset_gradient_tolerance() const
    {
        // Normalized by the field's slope s (OffsetPotential::level_set_slope()), so
        // front_conv_rel means the same thing whichever field is in use: |Phi - c| is a field
        // difference, not a length, and s is the conversion. s == 1 for `euclidean`, where this
        // is a no-op; for `smooth` Phi is a barrier with s ~ 1/delta, growing as the offset gets
        // finer, so without the factor rel silently asks for a tiny fraction of what it names.
        //
        // At the field's reference slope the bound reduces to |Phi - c| / s <= (rel / 2) * delta,
        // so rel 0.2 means "within 10% of target_distance"; elsewhere it is that bound relaxed by
        // s / |grad Phi|. The relaxation is deliberate, not slack: it is what lets the criterion
        // be met at a stationary point of Phi, where |grad Phi| -> 0 and no residual bound is
        // achievable at all. Two bodies close enough that Phi's minimum contour between them
        // exceeds c have no level set in the gap; the sheets converge onto that contour from
        // either side and rel controls how close they get. Expect `converged: true` beside a
        // plateaued max_phi_residual there -- that geometry being reported, not a placement
        // failure.
        const double s = m_offset_potential ? m_offset_potential->level_set_slope() : 1.;
        return std::max(
            m_offset_params.front_conv_rel * m_offset_params.target_distance * s * s,
            1e-16);
    }

    /**
     * @brief Stop the run if any reachable offset-surface vertex has left the potential's support.
     *
     * Beyond dhat, Phi is identically zero with a zero gradient: the vertex is given no direction
     * back, its residual saturates instead of growing, and the sizing field refines around a
     * vertex nothing can move. There is no recovery from that state and no honest report of it,
     * so it is a hard error, not a warning; the answer to it firing is a larger
     * offset_dhat_factor. Called once per optimization iteration and once on the band as built.
     */
    void check_offset_within_support(const char* when) const;

    /// How far vid is from the level set Phi = c, as a length. This is what the loop converges
    /// on and what the sizing field refines by.
    double band_vertex_residual(const size_t vid) const;

    /// Whether vid is an offset-surface vertex the optimizer could still place on the level set.
    /// Only the domain boundary disqualifies one: conservative growth ran out of room there, so
    /// the vertex is on the box because the offset could not extend past it, and no placement
    /// changes that. m_is_on_region must not disqualify it -- the flag is over-broad (splits
    /// propagate it, collapses OR it onto survivors), so a vertex carrying it may sit exactly
    /// where the offset wants it, and check_no_vertex_on_both_surfaces() already throws on the
    /// genuinely contradictory case.
    bool band_vertex_is_reachable(const size_t vid) const
    {
        return m_vertex_attribute[vid].on_bbox_faces.empty();
    }

    /// Which vertices lie on the offset surface. Shared by every measurement so they all agree
    /// on what "the band" is.
    std::vector<bool> band_vertex_mask() const;

    /// The furthest any offset-surface vertex sits from the input complex, by BVH. 0 when no
    /// offset exists yet. Sizes dhat in init_offset_potential().
    double max_band_vertex_distance() const;

    /// Sample the potential on the plane through the box centre normal to its shortest extent
    /// and write it as <path>_phi.vtu, n x n samples. See phi_grid_resolution; 0 disables.
    void write_phi_grid(const std::string& path, int n) const;

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
     * One definition of "inside this face", because two things are measured across a face: the
     * Phi residual (the diagnostic) and the placement gradient (the criterion). They must share
     * a lattice, or a run that fails the criterion in a face is not explicable by the residual.
     *
     * Interior lattice points of the barycentric subdivision at denominator n = k + 2: every
     * (i, j, l) with i + j + l = n and each >= 1, so nothing lands on an edge or a vertex, which
     * are measured separately and would otherwise be double-counted. n = k + 2 rather than 2D's
     * k + 1 because a triangle has no interior lattice point at denominator 2; k = 1 is then the
     * centroid, where a triangle inscribed in the offset is furthest from it.
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
     * The criterion cannot be a vertex criterion: a surface can have every vertex exactly on the
     * level set while its triangles cut across it, which reads as converged and is not the
     * offset -- the same gap the paper's normal-deviation criterion (Sec. 5.3.3) covered.
     * Sampling the face interior is what the potential makes possible: Phi is defined
     * everywhere, so the offset can be measured anywhere on the surface rather than only where
     * the mesh happens to have put a vertex. The same samples feed face_criterion_rel(), so the
     * sizing field refines a surface too coarse to represent the offset instead of letting it
     * decimate.
     *
     * Returns nothing for a face with an unreachable corner: a triangle running onto the input
     * complex is legitimately closer than target_distance across part of itself.
     */
    FaceSamples offset_face_samples(const Tuple& f) const;

    /// The per-face score the collapse and swap acceptance gates gate on (Collapse.cpp,
    /// Swap.cpp); >= 1 means the face fails the criterion. Also the per-face form of
    /// optimization_quality_stats(). It ranks nothing for refinement -- that is
    /// update_band_sizing_from_tolerance(), which measures the criterion directly.
    double face_criterion_rel(const Tuple& f) const;
    /// AMIPS of a cell over stop_energy -- the 3D twin of TriOptimizerMesh::quality_rel(), so
    /// both dimensions express "how bad is this element" on the same 1.0 scale.
    double cell_quality_rel(const size_t tid) const;
    /// ... and the worst of the (up to two) cells a face separates.
    double amips_rel_at_face(const Tuple& f) const;

    /**
     * @brief The offset surface's residual. Every band vertex and every face sample counts
     * toward the max and the average, pinned vertices included. The reachable/pinned split (see
     * band_vertex_is_reachable()) is attribution: a max driven by a pinned vertex calls for a
     * different construction, not more optimization.
     */
    struct DistanceSplit
    {
        double max_reachable = 0., avg_reachable = 0.;
        double max_pinned = 0.;
        size_t n_reachable = 0, n_pinned = 0;
        /// max_reachable, split by where it was measured: the vertex max says the surface is in
        /// the wrong place, the face max says it is too coarse to be in the right place. They
        /// call for different remedies -- smoothing versus refinement.
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
     * The convergence criterion. Structured like DistanceSplit and split the same three ways:
     *
     *  - reachable vs pinned -- only the reachable half gates the run; a pinned vertex with a
     *    large gradient is a construction defect, reported rather than iterated on.
     *  - measured vs skipped -- a vertex whose ring is inverted or unrounded is one the smoother
     *    refuses to place, so its gradient is not part of the fixed point. Counted and logged,
     *    never silently dropped: a run that "converges" over vertices it never measured has not.
     *  - at-vertex vs in-face -- E = (Phi(x) - c)^2 is a field, so it can be evaluated inside a
     *    face just as the residual is. Both count toward max_reachable, so a triangle whose
     *    corners sit on the level set while its interior chords across it fails the criterion.
     *    Split because at-vertex calls for smoothing and in-face for refinement.
     *
     * A face sample is not a variable an optimization step can move, so the convergence test
     * carries the in-face term and the Phase B stop test does not; see
     * phase_b_band_gradient_linf().
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
    /// @param include_face_samples false measures vertices only -- what Phase B's stop test
    /// wants, since a face sample is not a variable its sweeps can move. The convergence test
    /// always passes true.
    GradientSplit gradient_split(bool include_face_samples = true) const;
    /// Turn a residual_split()'s outside-support tally into the hard error. Separate from
    /// check_offset_within_support() so the per-iteration check can reuse a split it already has.
    void report_outside_support(const char* when, const DistanceSplit& s) const;

    /**
     * @brief Which tag the tets a swap creates should carry.
     *
     * A swap must not move the boundary between differently tagged regions, since that boundary
     * is the offset. Around an interior edge every incident tet already shares a tag, so this is
     * a cheap safety net there; on a surface flip the ring genuinely spans two and the majority
     * tag wins. Refuses the swap when three or more tags meet: there is then no single answer
     * and any choice would relabel a tet.
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
     * knows only the union of the two), to lower the order of a boundary vertex, or to collapse
     * across a feature of the input complex; it also records how badly aligned the offset surface
     * already was, so its `after` counterpart can tell a regression from a pre-existing defect.
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


    /// The offset's own collapse refusals, splitting the base's before_vertex bucket by reason.
    /// Same lifecycle as the base's CollapseRefusals: printed by the stall dumps, reset at each
    /// Phase A entry.
    struct OffsetCollapseRefusals
    {
        std::atomic<long> invariant{0}; ///< would put input-complex and offset on one vertex
        std::atomic<long> class_region{0}; ///< region vertex may not leave its surface
        std::atomic<long> class_offset{0}; ///< offset vertex may not leave its surface
        std::atomic<long> order2{0}; ///< order-2 vertex into lower order
        std::atomic<long> sublink{0}; ///< substructure_link_condition refusal
        void reset() { invariant = class_region = class_offset = order2 = sublink = 0; }
    };
    mutable OffsetCollapseRefusals m_offset_collapse_refusals;
    // The shared collapse merges the sizing scalar with min(removed, survivor) -- see
    // TetOptimizerMeshCollapse.cpp -- making every collapse a one-way refinement ratchet. That
    // is deliberate: a collapse must never discard a finer request the field has already made.

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
        bool is_edge_on_region = false;
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
    /// The construction label shared by every cell of the swap's ring, captured alongside the
    /// tag. Required: without it a swap leaves its new cells holding whatever label occupied the
    /// recycled tet slot.
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

    /// Whether edge `loc` lies on the INPUT complex. The base's is_edge_on_region() asks
    /// about the union of the two tracked surfaces, and is_edge_on_bbox() is the base's.
    bool is_edge_on_region(const Tuple& loc);
    bool is_edge_on_offset(const Tuple& loc);

    /**
     * @brief check that the ambient tag does not overlap with any other tags
     */
    bool ambient_assert();

    /**
     * @brief label input simplicial complex simplices, as defined in
     * m_offset_params.offset_selection
     */
    void label_input_complex();

    /// Set m_is_on_region from the input-complex labels. Called at the end of
    /// label_input_complex(), which is the earliest point at which the complex is known.
    void mark_input_complex_vertices();

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
     * @brief Phase B's placement of an offset-surface vertex: the offset energy alone, then
     * backtrack into the one-ring if the solved point inverts something.
     *
     * The only smoothing Phase B does -- smooth_before() refuses every other vertex there, so no
     * AMIPS term is assembled anywhere in the phase. Do not route it through the shared smoother;
     * measured worse -- see git history of this file. That objective is w_amips * AMIPS plus the
     * offset term, so the vertex rests where the two gradients cancel rather than on the level
     * set, and AMIPS being dimensionless its positional gradient scales as 1/h, which makes the
     * resting point worse as the mesh refines.
     *
     * A 1-D root find along the normal, Phi(x) = c, never a 3-D minimisation of the offset
     * energy: that energy's Hessian is rank one, so the level set's tangent plane is
     * unconstrained and the vertex slides. Element shape is left entirely to split/collapse/swap.
     * No incident tet may invert -- a constraint on the segment to the target, not a reason to
     * discard it: bisect for the furthest point along it that keeps every tet valid.
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
     * The base's SmoothRejectCounters covers the rejections inside the shared smoother; these
     * cover the sites before it, where the offset says no for its own reasons, plus the split of
     * what reached the smoother into offset-surface and interior.
     */
    struct SmoothTrace
    {
        std::atomic<int> attempted{0}; ///< smooth_before() entered
        std::atomic<int> before_bbox{0}; ///< base smooth_before said no: on the bounding box
        std::atomic<int> before_unrounded{0}; ///< base smooth_before said no: could not round
        std::atomic<int> before_on_region{0}; ///< input complex, frozen
        std::atomic<int> before_phase_b_not_offset{0}; ///< Phase B: wrong class for this sub-sweep
        std::atomic<int> before_phase_b_enveloped_background{
            0}; ///< Phase B: envelope-held, stencilled
        std::atomic<int> offset_attempted{0}; ///< reached the smoother carrying the offset term
        std::atomic<int> interior_attempted{0}; ///< reached it as an ordinary interior vertex

        void reset()
        {
            for (std::atomic<int>* c :
                 {&attempted,
                  &before_bbox,
                  &before_unrounded,
                  &before_on_region,
                  &before_phase_b_not_offset,
                  &before_phase_b_enveloped_background,
                  &offset_attempted,
                  &interior_attempted}) {
                c->store(0);
            }
        }
    };
    mutable SmoothTrace m_smooth_trace;

    void log_smooth_trace() const;

    /// Offset placements this Phase B pass that could not reach their unconstrained minimum: the
    /// root find entered its one-ring bisection, was refused outright by inversion, or found the
    /// ring already float-inverted on entry. Reset by phase_b_smooth() before each pass; a pass
    /// ending with this at zero is the loop's natural exit, since the potential is a frozen field
    /// and placements couple only through the ring constraint.
    mutable std::atomic<int> m_phase_b_constrained{0};

    /// Per-thread Newton solver for Phase B's interior AMIPS solves. Separate from the base's
    /// m_solver because it carries a different stopping rule: polysolve's rel_grad_norm_tol set
    /// to vertex_grad_tol_rel with a deep iteration budget, against the base's fixed shallow
    /// budget with no tolerance. Created on first use.
    mutable wmtk::threading::enumerable_thread_specific<
        std::unique_ptr<polysolve::nonlinear::Solver>>
        m_phase_b_solver;

    /**
     * @brief Which of four DISJOINT classes a vertex is accounted to when measuring movement.
     *
     * Bbox first, deliberately: a vertex may be on the bounding box and on a tracked surface at
     * once, and the bbox freeze in smooth_before() overrides everything else, so for "does it
     * actually move" the bbox decides. Offset before input is academic -- a vertex on both is a
     * construction defect check_no_vertex_on_both_surfaces() throws on.
     */
    enum class VClass { Bbox = 0, Offset = 1, Input = 2, Interior = 3, Count = 4 };

    VClass vertex_class(const size_t vid) const
    {
        if (!m_vertex_attribute[vid].on_bbox_faces.empty()) return VClass::Bbox;
        if (m_vertex_extra[vid].m_is_on_offset) return VClass::Offset;
        if (m_vertex_extra[vid].m_is_on_region) return VClass::Input;
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
     * be attempted constantly, accepted often, and still go nowhere in tiny steps. So
     * displacement is summed and maximised over accepted moves only -- a rejected move is
     * restored in place or rolled back, so its displacement is an intermediate the mesh never
     * kept. Smoothing is the only operation that moves an existing vertex (split inserts,
     * collapse merges at the survivor's position, swaps only rewire), so `accepted == 0` for a
     * class says exactly that no vertex of that class ever moved.
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
     * (1) Stale flags: on_bbox_faces is inherited or kept for a vertex not on that wall --
     *     collapse does not maintain it, which is safe only while wall vertices cannot move.
     * (2) Vacuous containment: the vertex is on the wall and smoothing does move it off, because
     *     the containment check found no tracked faces at it and so tested nothing.
     *
     * Distinguished at each accepted smoothing move by recording how many tracked faces the
     * vertex had and how much off-plane deviation the move introduced. Moves adding deviation
     * with zero tracked faces are (2); a violation with no such moves at all is (1).
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
     * @brief Seed the sizing field from the offset surface's current edge lengths, once, before
     * the first pass. Paper Sec. 5.3.3 Step 1: "initialized with the current length of each edge."
     *
     * Required: the field otherwise starts at the base target, far coarser than the offset
     * surface, so every offset edge is a collapse candidate on pass one and the surface is
     * decimated before any metric is computed.
     */
    void init_offset_sizing_field();

    /**
     * @brief How far the offset surface is from where it should be: {max, avg} over its vertices.
     *
     * The absolute error |dist(v, input complex) - target_distance|, over the band's outer
     * surface only -- a band cell meeting a plain-background cell, not the input complex it
     * wraps, whose interface sits at distance 0 by construction.
     *
     * A face with no opposite tet is on the domain boundary and must be counted, not skipped:
     * those vertices are pinned on the box and can never be fixed, so dropping them
     * under-reports a bbox-clipped offset by a large factor.
     */
    std::pair<double, double> compute_distance_deviation() const;

    /**
     * @brief The band's outer surface, recomputed live rather than read from the cached class.
     *
     * A band cell meeting a cell that is neither band nor input complex. Must be live: the
     * operations that ask run between one labelling pass and the next, so a face a split just
     * created carries whatever cached class its recycled slot happened to hold. Returns true for
     * a band face on the domain boundary; see compute_distance_deviation() for why.
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
     * A band face on the domain boundary means target_distance exceeded the clearance between
     * the input complex and the box, so the offset is clipped there. No operation can move those
     * vertices, making the target distance unreachable by construction -- worth saying out loud
     * rather than leaving as a mysterious plateau in max_dist_err.
     */
    void warn_if_offset_reaches_domain_boundary() const;

    /// The vertex compute_distance_deviation() last found the max at, and a dump of everything
    /// that could be stopping it from moving. Diagnostic only.
    mutable size_t m_worst_dist_vid = static_cast<size_t>(-1);
    void log_worst_dist_vertex() const;

    /**
     * @brief Diagnostic: dump the single worst-quality tet and everything that could be pinning
     * it, so a Phase A that will not converge says which element and why rather than only a max.
     *
     * Reports, for the max-cbrt(cell_quality) tet: its volume and inversion state, its region
     * label, and per vertex the flags that decide whether any operation may touch it -- on the
     * input complex, on the offset surface, on the bounding box, rounded, order and sizing
     * scalar. Then each of the six edges against the two length gates, so it is visible whether
     * split or collapse would even consider them: split needs len^2 >= splitting_l2 * sbar^2,
     * and the coarsening pass stops at len^2 <= collapsing_l2 unless coarsen_unbounded. The
     * point is to separate "no operation is attempted here" from "operations are attempted and
     * refused", which the aggregate counters cannot distinguish.
     */
    void log_worst_tet(const char* when) const;

    /// {max_dist_err, avg_dist_err, max_phi_residual, avg_phi_residual, max_grad, avg_grad} per
    /// optimization iteration. Only the last pair is the convergence criterion; the rest are
    /// diagnostics answering different questions -- the Euclidean error says how far the smoothed
    /// offset ended up from the exact one, and the Phi residual still ranks the sizing field.
    std::vector<std::array<double, 8>> optimization_metrics;
    /// {split-born vertices, recollapsed, recollapsed in the immediately following collapse
    /// pass} per A/B round, in step with op_counts. See VertexExtra::m_born_epoch.
    std::vector<std::array<int, 3>> churn_counts;
    /// {splits, collapses, swaps} per A/B round -- one entry per round the driver runs, the
    /// converging one included, as deltas rather than running totals. Phase B does no topological
    /// work, so a round's entry is exactly what its Phase A did. Does not mirror
    /// optimization_metrics, which is a single whole-run summary.
    std::vector<std::array<int, 3>> op_counts;
    /// Whether the optimization met both convergence criteria before the iteration cap.
    bool m_converged = false;

    /// Per-iteration operation counters, reset before each iteration's operation passes.
    /// Churn: split-born vertices a collapse later removed, and the subset removed in the same
    /// pass-pair that created them. The rest survived at least into a later iteration.
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
    std::atomic<int> iter_cnt_split_offset_base_reject{0};
    std::atomic<int> iter_cnt_split_offset_tried{0};
    std::atomic<int> iter_cnt_split_offset{0};
    std::atomic<int> iter_cnt_split_offset_reject{0};


    //// collapse


    //// sizing field
    // The sizing field's bounds are user-configurable: see Parameters::min_sizing_scalar /
    // max_sizing_scalar / sizing_gradation. Refinement stops at min_sizing_scalar; coarsening
    // never exceeds max_sizing_scalar, since a sizing scalar above 1 would be coarser than the
    // target edge length the user asked for.

    /**
     * @brief unsigned mean ratio metric of a triangle: 2*sqrt(3)*area / (sum of squared edge
     * lengths). 1 for equilateral, -> 0 as the triangle degenerates.
     */
    static double mean_ratio_metric(const Vector3d& p0, const Vector3d& p1, const Vector3d& p2);
    //// sizing field

    //// swap

    /**
     * @brief execute simplistic marching tets. All edges with one vertex labelled 0 and the other
     * 1/2 are split, at the midpoint.
     *
     * Do not re-add a distance-field placement that root-finds each band-boundary edge onto
     * d(x) = delta at insertion time; measured worse -- see git history of this file. The paper
     * places inserted vertices at the midpoint (Sec. 5.2) and leaves the distance entirely to
     * the Step-3 optimization, which has error feedback and the shared accept checks.
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

        bool is_edge_on_region = false;
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
