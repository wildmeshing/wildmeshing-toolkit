#pragma once
#include <wmtk/TriMesh.h>
#include <wmtk/TriOptimizerMesh.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include "OffsetPotential.hpp"
#include "Parameters.h"
#include "SimplicialComplexBVH.hpp"
#include "TagEnvelopes.hpp"

using CellTag = std::set<int64_t>;

namespace wmtk::components::topological_offset {


const int64_t TEMP_OFFSET_TRI_TAG = -1;
const CellTag TEMP_OFFSET_TRI_TAG_SET{TEMP_OFFSET_TRI_TAG};

/**
 * @brief Per-vertex data the shared 2D optimizer knows nothing about.
 *
 * Position, rounding, bbox membership, sizing and partition all live on
 * wmtk::TriOptimizerMesh::VertexAttributes. The three flags here say WHICH of the tracked
 * surfaces a vertex belongs to; the base's m_is_on_surface is their union. They are not mutually
 * exclusive -- a vertex where the offset boundary meets another region's boundary carries both.
 */
class VertexExtra2d
{
public:
    int label = 0;
    bool m_is_on_input = false; // on the input complex
    bool m_is_on_offset = false; // on the offset boundary itself
    bool m_is_on_region = false; // on some OTHER tag region's boundary

    /**
     * @brief WHICH tag boundaries this vertex lies on -- one bit per input tag, ambient
     * included. See TopoOffsetTriMesh::m_tag_envelopes for what the bits dispatch to.
     *
     * Seeded in init_surfaces_and_boundaries() from the input partition, then propagated by
     * the operations: a split's new vertex takes the AND of its endpoints (it lies on a
     * boundary only if the whole edge did), a collapse's survivor takes the OR (it now carries
     * both vertices' geometry). The 3D twin is VertexExtra::m_boundary_mask.
     */
    uint64_t m_boundary_mask = 0;

    /// CHURN INSTRUMENTATION: which split pass created this vertex, from
    /// wmtk::TriOptimizerMesh::m_op_epoch. 0 means "not created by an optimization split".
    /// Read only by collapse_after_vertex(). ASSIGNED at each split, never OR'd -- a recycled
    /// slot carries a dead vertex's epoch. The 3D twin is VertexExtra::m_born_epoch.
    uint32_t m_born_epoch = 0;
};


/// Per-edge construction label; the surface tags themselves are the base's
/// wmtk::SurfaceTagAttributes. Registered with m_edge_attr_group.
class EdgeExtra2d
{
public:
    int label = 0;
};


/// Per-face construction label. The region tag lives in the base's FaceAttributes::tags.
/// Registered with m_face_attr_group.
class FaceExtra2d
{
public:
    int label = 0;
};


/**
 * @brief The offset's 2D mesh, on the shared 2D optimizer.
 *
 * Mirrors TopoOffsetTetMesh: the construction phase is entirely its own, and the optimization
 * phase that follows is wmtk::TriOptimizerMesh's.
 *
 * Two surfaces are tracked, not three, and not one. Every tag-region boundary -- the input
 * complex and the domain wall included -- keeps the primary class 0 and is held inside its tags'
 * envelopes, exactly as triwild holds its input. The offset boundary is OFFSET_SURFACE_CLASS: in
 * 2D it is exactly the set of edges across which the incident FACE LABELS differ -- there is no
 * stored definition of it, it falls out of the labelling, which is why label_offset_boundary()
 * derives it once at the top of the optimization. Class-0 edges may move, within their tubes;
 * only the offset one is driven toward target_distance.
 */
class TopoOffsetTriMesh : public wmtk::TriOptimizerMesh
{
public: // mode for splitting in marching tets
    enum class EdgeSplitMode {
        Midpoint = 0, // construction: simplicial embedding AND marching_tris
        Optimization = 2 // the optimization phase; the shared engine places the vertex
    };

public:
    int m_vtu_counter = 0;
    std::array<size_t, 3> m_init_counts = {{0, 0, 0}};
    size_t m_tags_count;
    /**
     * @brief The input complex as loaded. BUILT ONCE, NEVER REBUILT.
     *
     * The EUCLIDEAN distance to the input, now a diagnostic rather than the definition of the
     * offset -- see m_offset_potential, which is what the optimization is driven by.
     * init_input_complex_bvh() has exactly one call site, in the driver, before execute_offset()
     * runs -- so this holds the original geometry however the mesh elements representing the
     * complex are later refined, coarsened or smoothed.
     *
     * That invariant is load-bearing and easy to break by accident. Since the complex stopped
     * being frozen, rebuilding this from the live mesh would redefine the offset distance in
     * terms of a surface the optimizer had just moved, and the convergence criterion would then
     * be measuring the mesh against itself.
     *
     * THE ONLY STRUCTURE OVER THE INPUT COMPLEX. For offset_field "euclidean" the potential
     * shares this very object as its query engine (value and nearest-feature both), which is
     * why it is a shared_ptr. There used to be a second one -- an exact-kind SampleEnvelope
     * over the same segments, built again inside init_offset_potential() -- duplicating the
     * geometry for one consumer; deleted. Nothing ever needed it for CONTAINMENT either: the
     * input complex is held in place by the per-tag region envelopes (m_tag_envelopes), like
     * every other region boundary.
     */
    std::shared_ptr<SimplicialComplexBVH> m_input_complex_bvh;

    /**
     * @brief The SMOOTH OFFSET POTENTIAL, and with it the definition of the offset itself.
     *
     * The offset boundary is the level set Phi = c. Built from the SAME extraction as
     * m_input_complex_bvh, in the same call, so the two describe the same geometry and the
     * identical never-rebuilt invariant applies. See OffsetPotential for what Phi is and why
     * the Euclidean distance was not usable as the thing to minimise.
     *
     * shared_ptr because OffsetEnergy2D holds one per smoothing call.
     */
    std::shared_ptr<OffsetPotential2D> m_offset_potential;

    /**
     * @brief ONE CONTAINMENT ENVELOPE PER INPUT TAG, ambient included. Both phases.
     *
     * E_t is a tube of half-width m_envelope_eps around the boundary segments of region t as
     * the INPUT mesh carried them -- built in init_surfaces_and_boundaries(), before offset
     * construction, so the band's later tag rewriting never enters them. A simplex on several
     * boundaries is constrained by the INTERSECTION of its tags' tubes (envelope_for_mask()),
     * which pins junction points to the junction itself -- including an input complex made of
     * isolated points, which only arises where two or more selected tags meet.
     *
     * That intersection is what replaced the single fused region-boundary envelope and the
     * `region_envelope_from_input` switch that chose when to build it. Building it from the
     * input is now the only behaviour, and it is the right one for the same reason the switch
     * defaulted to true: the band's tags REPLACE a face's own, so a region the band grows
     * through loses its tag there and an envelope built afterwards is a tube around a curve
     * truncated at the band.
     *
     * m_envelope (the base's pointer) survives as a UnionEnvelope over these members, purely so
     * the shared engine's direct uses of it -- the collapse_edge_before point check, and the
     * "segment does not exist yet" fallback in surface_envelope_for_edge() -- keep union
     * semantics.
     *
     * INTERIOR EDGES OF A REGION ARE NOT HELD BY THESE: an edge interior to a region has
     * identical tag sets on both sides and lands in no bucket, so a filled complex's interior
     * is free to optimise.
     */
    std::map<int64_t, std::shared_ptr<SampleEnvelope>> m_tag_envelopes;

    /**
     * @brief The per-tag boundary POLYLINE, with the adjacency an arclength walk needs.
     *
     * The same segments m_tag_envelopes[tag] was built from, kept in the same indexing, so
     * SampleEnvelope::nearest_point_feature()'s `feature_id` (a polyline VERTEX index when
     * on_corner, else a SEGMENT index) indexes straight into these. Built once beside the
     * envelopes in init_surfaces_and_boundaries(), from the INPUT partition, and never rebuilt
     * -- the envelope is a tube around this curve, so the two must not be able to drift apart.
     *
     * WHY THE COMPONENT KEEPS ITS OWN COPY. SampleEnvelope has the arrays (m_v2 / m_e2) but they
     * are protected, and this mesh is not a subclass of it. The buckets are already in hand at
     * the build site, so copying them costs nothing and needs no shared-code change.
     *
     * `at_vertex` is what the envelope cannot answer: which segments meet at a polyline vertex,
     * i.e. how the walk continues across a corner. Usually two; one at an open end; more where
     * three curves meet, and there the walk stops because the continuation is ambiguous.
     */
    struct TagPolyline2d
    {
        std::vector<Eigen::Vector2i> E; ///< segments, indexing m_env_polyline_V
        std::vector<std::vector<int>> at_vertex; ///< polyline vertex -> incident segment ids
    };
    /// Shared vertex array for every TagPolyline2d: the positions as init_surfaces_and_boundaries()
    /// saw them, indexed by the mesh vid AT CONSTRUCTION. Never renumbered -- the envelopes hold
    /// the same snapshot, and both describe the input, not the live mesh.
    std::vector<Eigen::Vector2d> m_env_polyline_V;
    std::map<int64_t, TagPolyline2d> m_tag_polyline;

    /// Input tag id -> bit position in VertexExtra2d::m_boundary_mask. Assigned in
    /// init_from_image() once the tag maps are complete; at most 64 input tags.
    std::map<int64_t, int> m_tag_bit;

    /// Memoized IntersectionEnvelope per multi-bit mask. Lazily built under the mutex because
    /// the queries that need them run concurrently under kPartition.
    mutable std::map<uint64_t, std::shared_ptr<SampleEnvelope>> m_isect_cache;
    mutable std::mutex m_isect_mutex;

    /**
     * @brief Memoized "region tubes AND the offset envelope", keyed by the region mask.
     *
     * A SIMPLEX CAN BE ON BOTH, and until 2026-08-23 the dispatch made that an either/or: a
     * vertex on the offset front that also lay on tag boundaries got its region tubes and LOST
     * the Phase A offset envelope, while an offset EDGE whose two endpoints happened to share a
     * region bit got the region tube and lost the offset one. Both are now the intersection.
     *
     * SEPARATE FROM m_isect_cache because the members differ in lifetime: the tag envelopes are
     * built once from the input mesh and live for the whole run, while m_offset_envelope is
     * rebuilt after every Phase B. rebuild_offset_envelope() clears this, and must keep doing
     * so -- an entry here holds a shared_ptr to the PREVIOUS round's offset tube and would
     * silently pin the boundary to where it was two rounds ago. Guarded by m_isect_mutex.
     */
    mutable std::map<uint64_t, std::shared_ptr<SampleEnvelope>> m_offset_isect_cache;

    /**
     * @brief The containment a simplex with this region mask, on/off the offset front, must
     * satisfy -- the INTERSECTION of everything that holds it, or null if nothing does.
     *
     * The single place the two containment families are composed. `region_mask` dispatches
     * through envelope_for_mask() (itself an intersection when the mask is multi-bit, which is
     * what pins a junction to the junction); `on_offset` adds m_offset_envelope, but only in
     * Phase A -- Phase B is the pass whose job is to move the offset boundary, so there it
     * contributes nothing and the result is the region tubes alone.
     */
    std::shared_ptr<SampleEnvelope> containment_for(uint64_t region_mask, bool on_offset) const;

    /**
     * @brief Move `x` back inside every region tube this vertex lies on. True if it ended up
     * inside all of them.
     *
     * THE PROJECTION HALF OF THE PROJECTED-GRADIENT PLACEMENT. An offset vertex that a region
     * envelope also holds takes the SAME unconstrained Gauss-Seidel step on the SAME objective
     * as any other offset vertex, and then this restores its validity -- rather than the step
     * itself being refused whenever it would leave the tube.
     *
     * Why the change (2026-08-23): refusal cannot express "go as far as you may". On
     * topo_annots_groups at convergence_gradient_norm_rel 1e-3, four junction vertices reported
     * `EnvelopeBlocked` in EVERY pass of EVERY round -- every trial length along the descent
     * direction left the tube, so they never moved at all, and one of them (v208, Phi/c = 5.6)
     * set max_grad at 3.61 against a bar of 0.0339 for all ten rounds while 115k operations ran
     * around it. Projecting instead keeps the component of the step the tube allows, which for a
     * vertex ON a boundary curve is motion ALONG that curve -- exactly the freedom it has.
     *
     * NEVER ASKS A COMPOSITE. nearest_point() and squared_distance() are non-virtual on
     * SampleEnvelope and would bind to an IntersectionEnvelope's base subobject, whose BVH was
     * never built (TagEnvelopes.hpp). So this walks the mask's REAL members from
     * m_tag_envelopes, the way smoothing_energy_envelope() picks its pull target, and composes
     * them itself by ALTERNATING PROJECTION: repeatedly project onto the member it violates
     * worst. Each nearest_point() lands x exactly on that member's curve, so one round settles a
     * single tube and the remaining rounds are what converge a junction onto the intersection of
     * its curves.
     *
     * Returns false if the alternation did not converge, which is the caller's signal to keep
     * the entry position rather than commit an invalid one.
     */
    bool project_into_containment(size_t vid, Vector2d& x) const;

    /**
     * @brief Which tag's boundary curve a vertex slides along, or -1.
     *
     * The bit of its mask whose curve passes CLOSEST to it. For the common multi-bit case that
     * choice is immaterial: a mask carries a bit per tag on EITHER side of the boundary (the
     * symmetric difference init_surfaces_and_boundaries() classifies by), so an interface
     * between two regions gives both bits and both curves contain that same interface. Where
     * the curves genuinely differ the line search still tests containment against EVERY member
     * tube, so picking the nearest only chooses the parameterization, never the constraint.
     */
    int64_t tangent_curve_tag(size_t vid, const Vector2d& x) const;

    /**
     * @brief March `s` of arclength along tag `tag`'s boundary polyline from `x`'s foot on it.
     *
     * The reduced coordinate of the tangential placement: the constraint is eliminated rather
     * than enforced, so every point this returns is ON the curve and needs no containment test
     * of its own. Returns false if the walk cannot continue -- an open end, or a polyline vertex
     * where three or more segments meet and the continuation is ambiguous -- in which case `out`
     * holds the furthest point reached and the caller treats it as the end of the feasible
     * interval.
     *
     * CORNERS ARE NOT SPECIAL-CASED, deliberately. The walk crosses any corner; what stops a
     * vertex from sliding past a sharp one is the INCIDENT CHORD leaving its tube, which the
     * caller's backtracking finds. That reproduces the true bound -- roughly eps/sin(theta) for
     * a turn of theta, unbounded along a straight run -- with no angle threshold anywhere, and
     * an angle threshold is exactly what a corner special case would have needed.
     */
    bool walk_along_curve(int64_t tag, const Vector2d& x, double s, Vector2d& out) const;

    /**
     * @brief The unit tangent of tag `tag`'s curve at `x`'s foot, or false if there is none.
     *
     * From nearest_point_feature()'s seg_normal, rotated a quarter turn. At a polyline vertex
     * (on_corner) the tangent is two-valued, so this takes the incident segment whose direction
     * best aligns with the descent the caller is about to attempt -- the standard reading of a
     * one-sided derivative at a kink.
     */
    bool curve_tangent(int64_t tag, const Vector2d& x, const Vector2d& prefer, Vector2d& tau) const;

    /**
     * @brief Which half of the alternating optimization is running.
     *
     * THE TWO CRITERIA ARE OPTIMIZED IN TURN, NOT JOINTLY. This is the 2D counterpart of
     * TopoOffsetTetMesh::OptPhase, and its declaration carries the measurements that motivated
     * the split in 3D. 2D's joint loop did converge on all three examples, so the case here is
     * weaker in kind but the same in shape: on topological_offset_2d the residual descends to
     * 1.25x tolerance by iteration 9 and then spends four more iterations moving 1.25 -> 1.03
     * while the mesh grows 11460 -> 11566 vertices, converging at 0.978x -- a pass with 2% of
     * margin. Element quality is never the binding criterion on that run (AMIPS holds at 0.39x
     * of stop_energy throughout), so the two criteria are not sharing the loop so much as one is
     * paying for the other's operations.
     *
     * PHASE A is TriWild, and nothing else. Same operations, same gates, same sizing field, same
     * stall-driven refinement -- the offset contributes no energy term, no acceptance criterion
     * and no stop metric. Its one addition is m_offset_envelope.
     *
     * PHASE B moves the offset boundary and nothing else: smoothing passes against the offset
     * energy, run to a fixed point, with no envelope on the offset (it is what has to travel)
     * and no topological operations at all.
     *
     * The sizing field is SHARED and both phases write it: Phase A through TriWild's own stall
     * refinement on element quality, Phase B through the Phi residual of the faces smoothing
     * could not place.
     */
    enum class OptPhase { A, B };

    /// Which phase is running. Read by every hook that differs between them; see OptPhase.
    OptPhase m_phase = OptPhase::A;

    /**
     * @brief Phase B's two sub-sweeps, run in this order within every smoothing pass.
     *
     * Offset: place every offset-boundary vertex on the level set, by the 1-D minimization of
     * (Phi - c)^2 along grad Phi, backtracked into the one-ring if the minimum lies outside it.
     * Background: relax the interior by AMIPS, under the boundary those placements just defined.
     *
     * SEPARATE SWEEPS, NOT ONE INTERLEAVED PASS. Relaxing the background against the PREVIOUS
     * pass's boundary spends the work on a configuration that is about to move, and interleaving
     * makes each vertex's one-ring a moving target for its neighbours within the same sweep --
     * so the backtracking a placement hits depends on the visit order rather than on the
     * geometry. Ordering them makes the pass mean "place, then relieve what the placement cost".
     *
     * smooth_before() reads this and refuses the other class outright, so the two sweeps share
     * all of the executor, counters and accept gates and differ only in who they admit.
     */
    enum class PhaseBSub { Offset, Background };
    PhaseBSub m_phase_b_sub = PhaseBSub::Offset;

    /// DIAGNOSTIC: set by the driver after the first round when
    /// ab_no_collapse_after_first_round is on, and read by collapse_edge_before().
    bool m_ab_collapses_disabled = false;

    // NO offset_criterion_gates_operations(). It gated a non-degrading Phi test on every
    // collapse and swap in Phase B, which is TETWILD PARITY'S opposite: the envelope is the
    // constraint and the criterion belongs to Phase B's own placement. What survives is the
    // COARSENING bar, applied where 3D applies it -- absolute, and only in m_coarsen_mode.

    /**
     * @brief The tube the offset boundary may not leave during Phase A.
     *
     * BUILT WHEN THE OFFSET IS, by optimize_offset() right after label_offset_boundary(), and
     * REFRESHED AT THE END OF EVERY PHASE B from the boundary as that phase just left it, with
     * eps = ab_offset_envelope_rel x target_distance. Refreshing is what lets the boundary still
     * travel across rounds: each Phase A pins it near its current position, and each Phase B is
     * free to move it somewhere the next Phase A will then pin.
     *
     * NON-NULL FOR THE WHOLE RUN once the offset exists -- it is NOT nulled on entering Phase B,
     * which is how this used to say "not applicable". The phase test lives in containment_for()
     * instead, so the pointer answers "has an offset been built" and the phase answers "does its
     * tube constrain right now"; conflating the two meant a reader outside Phase A could not
     * tell them apart.
     *
     * m_tag_envelopes, the input complex's and the other regions', are built once from the input
     * mesh and never rebuilt -- that geometry is what the offset distance is measured against.
     */
    std::shared_ptr<SampleEnvelope> m_offset_envelope;

    /// Rebuild m_offset_envelope from the current offset-boundary segments, and drop the
    /// intersections memoized against the old one. Called once when the offset is created and
    /// again at the end of every Phase B.
    void rebuild_offset_envelope();

    /// Hard error if any vertex is on BOTH the input complex and the offset boundary -- a state
    /// no placement satisfies. Called at construction and after every phase; see the definition
    /// for why a collapse can create it out of two individually fine vertices.
    void check_no_vertex_on_both_surfaces(const char* when) const;

    /// The A/B driver: Phase A (TriWild + offset envelope), Phase B (smoothing to a fixed point,
    /// then refine where Phi is stuck), repeated until both criteria are inside tolerance or
    /// ab_max_rounds is reached. Replaces the single mesh_improvement() call.
    void optimize_offset_alternating();

    /// Phase B's smoothing loop. Each pass sweeps every vertex once: offset vertices are placed
    /// on the level set by smooth_offset_vertex_backtracking(), background (interior) vertices
    /// minimize their one-ring AMIPS by smooth_interior_vertex_phase_b() -- each visit runs its
    /// vertex's local solve under the run's own gradient bar (offset placement) or to
    /// ab_vertex_grad_tol_rel of its entry gradient (interior AMIPS).
    /// Returns the number of passes run; the natural exit is a pass in which NO offset vertex
    /// was backtracked by its one-ring (m_phase_b_constrained == 0). See the definition for the
    /// other exits (the run's own convergence bar, the no-progress guard, and an optional
    /// ab_smooth_max_passes cap).
    size_t phase_b_smooth();

    /**
     * @brief Phase B's placement of an offset-boundary vertex: the 1-D minimization of
     * E = (Phi - c)^2 along grad Phi, Levenberg-Marquardt damped, backtracked into the one-ring
     * by bisection if the minimum would invert it.
     *
     * See the definition, and the 3D twin it is a transcription of, for why this is not a 3-D
     * (here 2-D) minimization of E -- the Gauss-Newton Hessian 2 g g^T is RANK ONE, so it fixes
     * one degree of freedom and lets the tangential one drift -- and for why the step is damped
     * rather than the plain Gauss-Newton step, which is singular exactly at a pinch minimum.
     */
    bool smooth_offset_vertex_backtracking(const Tuple& t);

    /**
     * @brief Phase B's placement of a background vertex: Newton on the one-ring AMIPS energy,
     * to this vertex's own minimum.
     *
     * The counterpart of smooth_offset_vertex_backtracking() for the vertices that carry no
     * surface. Solving the interior alongside the boundary is what opens one-rings that would
     * otherwise force the offset root find to backtrack -- which is the pass loop's exit
     * criterion. Reuses the shared smoother (optimization::smooth_vertex_2d) with Phase B's own
     * solver, configured to stop at ab_vertex_grad_tol_rel of the visit's initial gradient.
     */
    bool smooth_interior_vertex_phase_b(const Tuple& t);

    /**
     * @brief L-inf over offset vertices of |grad (Phi - c)^2| -- the gradient of the same
     * objective Phase B's sweeps minimize. Exactly 0 at the Gauss-Seidel fixed point, whatever
     * the residual, so it distinguishes "placement finished" from "placement blocked": a vertex
     * whose move is refused contributes zero DISPLACEMENT but full gradient.
     *
     * VERTICES ONLY, exactly as the convergence test itself is: the criterion gates on the
     * variables the optimizer owns, and the edge-interior samples are a resolution
     * diagnostic. See gradient_split().
     */
    double phase_b_band_gradient_linf();

    /**
     * @brief Per-vertex band sizing update, run after Phase B. Returns the number changed.
     *
     * REFINE-ONLY, AND ONLY ON PURE CHORD ERROR. "In tolerance" is ||grad (Phi - c)^2|| <=
     * offset_gradient_tolerance() -- at band vertices that IS the convergence criterion, and
     * at edge-interior samples (the one lattice for_each_offset_edge_sample() defines) it is
     * the same quantity continued into the chords: the diagnostic the criterion reports but
     * does not gate on, and exactly what refinement can fix.
     *
     *  - HALVE when the vertex and every boundary one-ring neighbour are in tolerance but some
     *    sample inside an incident band edge is not. The boundary passes through the right
     *    places and the chord between them still cuts the level set: pure resolution error, the
     *    one thing a finer sizing field can actually fix.
     *  - Otherwise leave it alone. A vertex that is itself out of tolerance is MISPLACED, not
     *    under-resolved.
     *
     * THIS IS THE ONLY THING THAT REFINES THE SIZING FIELD FOR THE OFFSET. It replaced
     * refine_sizing_where_phi_is_stuck(), which ranked SEGMENTS by a max-of-three score and
     * force-split the worst ones' edges; that routine is deleted, exactly as in 3D. Phase A
     * keeps TriWild's own quality-driven stall response (refine_sizing_around_worst), which
     * ranks ELEMENTS by AMIPS and is a different question -- the two write the same field.
     */
    size_t update_band_sizing_from_tolerance();

    EdgeSplitMode m_edge_split_mode = EdgeSplitMode::Midpoint;

    // tag name maps
    std::map<std::string, int64_t> m_tag_name_to_id;
    std::map<int64_t, std::string> m_tag_id_to_name;
    CellTag m_offset_output_tag_ids;

    // if in 'singlebody' mode
    bool m_singlebody = false;
    int64_t m_single_tag;

    // just for retaining in output. dont actually use
    bool m_has_envelope = false;
    MatrixXd m_V_envelope;
    MatrixXi m_F_envelope;

    /**
     * @brief SurfaceTagAttributes::m_surface_class: which of the TWO tracked surfaces an edge
     * belongs to. Same scheme as 3D.
     *
     * OFFSET is the surface the optimization is trying to place at target_distance. Everything
     * else -- the input complex, another body's outline, an overlap seam, the domain wall --
     * keeps the primary class 0 and is envelope-checked by the shared operations exactly as in
     * triwild and simwild. The distinction has to exist: filing a region boundary under OFFSET
     * is what sent the offset placement at vertices sitting ~40x target_distance from the input
     * complex, and what left the sizing field refining around them forever. Splitting class 0
     * further into INPUT and REGION, as this used to, bought nothing once the per-tag envelopes
     * arrived -- the boundary MASK says which tubes hold a simplex, and it says it per tag.
     */
    static constexpr int INPUT_SURFACE_CLASS = 0;
    static constexpr int OFFSET_SURFACE_CLASS = 1;

    /// The base holds only wmtk::OptimizerParameters; this is the same object, typed.
    Parameters& m_offset_params;

    using VertexExtraCol = wmtk::AttributeCollection<VertexExtra2d>;
    using EdgeExtraCol = wmtk::AttributeCollection<EdgeExtra2d>;
    using FaceExtraCol = wmtk::AttributeCollection<FaceExtra2d>;
    // m_vertex_attribute, m_edge_attribute and m_face_attribute are the base's; these three
    // are registered alongside them in its attribute groups.
    VertexExtraCol m_vertex_extra;
    EdgeExtraCol m_edge_extra;
    FaceExtraCol m_face_extra;

    TopoOffsetTriMesh(Parameters& _m_offset_params, int _num_threads = 0)
        : wmtk::TriOptimizerMesh(_m_offset_params)
        , m_offset_params(_m_offset_params)
    {
        NUM_THREADS = _num_threads;
        m_vertex_attr_group.add(&m_vertex_extra);
        m_edge_attr_group.add(&m_edge_extra);
        m_face_attr_group.add(&m_face_extra);

        // As in 3D. The per-vertex Newton solver logs a line per smoothing attempt at info
        // level, which is one line per vertex per pass: on a 13k-vertex mesh over 80 iterations
        // that is a 1.9 GB log with the run's own output buried in it.
        optimization::deactivate_opt_logger();
    }

    ~TopoOffsetTriMesh() override = default;

    /**
     * @brief Place a vertex, keeping its exact and rounded coordinates in step.
     *
     * As in 3D: the offset works in doubles (its construction is driven by a BVH distance
     * field), so every vertex it places is rounded, but m_pos must still be filled because the
     * shared split's exact-midpoint fallback reads it.
     */
    void set_vertex_position(const size_t vid, const Vector2d& p)
    {
        m_vertex_attribute[vid].m_posf = p;
        m_vertex_attribute[vid].m_pos = to_rational(p);
        m_vertex_attribute[vid].m_is_rounded = true;
    }

    /// Whether edge `eid` is on the offset boundary / carries input geometry / bounds some other
    /// tag region.
    bool edge_is_offset(const size_t eid) const
    {
        return m_edge_attribute[eid].m_is_surface_fs &&
               m_edge_attribute[eid].m_surface_class == OFFSET_SURFACE_CLASS;
    }
    /// ... and whether it bounds a REGION -- any tracked edge that is not the offset boundary.
    /// The input complex is included, and deliberately: both are held by the same per-tag
    /// envelopes and neither is what the optimization moves. Same shape as 3D's face_is_region().
    bool edge_is_region(const size_t eid) const
    {
        return m_edge_attribute[eid].m_is_surface_fs &&
               m_edge_attribute[eid].m_surface_class != OFFSET_SURFACE_CLASS;
    }
    /**
     * @brief An edge's / face's shared attributes together with the offset's own label.
     *
     * The marching-triangles splits snapshot a simplex and write it back onto the pieces it
     * became, and both halves have to travel together.
     */
    struct EdgeSnapshot2d
    {
        EdgeAttributes tags;
        EdgeExtra2d extra;
    };
    struct FaceSnapshot2d
    {
        FaceAttributes attrs;
        FaceExtra2d extra;
    };
    EdgeSnapshot2d edge_snapshot(const size_t eid) const
    {
        return EdgeSnapshot2d{m_edge_attribute[eid], m_edge_extra[eid]};
    }
    void restore_edge(const size_t eid, const EdgeSnapshot2d& s)
    {
        m_edge_attribute[eid] = s.tags;
        m_edge_extra[eid] = s.extra;
    }
    FaceSnapshot2d face_snapshot(const size_t fid) const
    {
        return FaceSnapshot2d{m_face_attribute[fid], m_face_extra[fid]};
    }
    void restore_face(const size_t fid, const FaceSnapshot2d& s)
    {
        m_face_attribute[fid] = s.attrs;
        m_face_extra[fid] = s.extra;
    }

    /**
     * @brief Tag the two tracked surfaces for the optimization phase.
     *
     * The offset boundary in 2D has no stored definition -- it is exactly the set of edges
     * across which the incident FACE LABELS differ, so it falls out of the labelling and is
     * recomputed here once. An edge with only one incident face is on the domain boundary and
     * is tagged bbox instead, which is what stops the box from collapsing.
     */
    void label_offset_boundary();

    /**
     * @brief Whether face `fid` belongs to the closed offset region, read from its LABEL.
     *
     * The region is the offset band (label 2) plus the input complex it wraps (label 1). Both
     * are set at construction -- the complex by label_input_complex(), the band by
     * marching_tris() -- from geometry, not from tags, and every operation now carries the label
     * onto the faces it creates, so this is exact.
     *
     * It used to read the TAGS, which cannot express the distinction the label exists for:
     * the band is named by the output tag, and nothing stops that tag already appearing
     * somewhere else in the input mesh. Every such face would then read as offset band, its
     * boundary would classify as OFFSET rather than REGION, and the offset smoothing energy
     * would drag an unrelated region toward the level set.
     */
    bool face_in_region(const size_t fid) const;

    /// Whether face `fid` is part of the INPUT complex (as opposed to the offset band).
    /// Label 1, assigned by label_input_complex() from the user's selection expression.
    bool face_is_input_complex(const size_t fid) const;

    /**
     * @brief The substructure the link condition is evaluated against, DERIVED not cached.
     *
     * substructure_link_condition() asks these, and it is only as good as the answers. The
     * cached edge tags are refreshed once per iteration, which is too coarse: the split pass
     * runs first and creates edges the tagging never classified, so the collapse pass that
     * follows evaluates the condition against a substructure that no longer describes the
     * mesh -- which is exactly why split and collapse tear the region together while each is
     * safe alone.
     *
     * Computing them from the face labels on demand costs a face lookup and cannot go stale.
     */
    bool vertex_is_on_surface(const size_t vid) const override;
    bool edge_is_on_surface(const std::array<size_t, 2>& vids) const override;

    /// The 2D optimization phase: split / collapse / swap / smooth on the shared driver.
    void optimize_offset(const std::filesystem::path& output_file);

    /// Whether this face carries one of the offset output tags, i.e. is inside the offset band.
    /// Read from the tags, which every shared operation maintains, rather than from the face
    /// label, which is only refreshed once per optimization iteration.
    bool face_is_offset_band(const size_t fid) const;


    /**
     * @brief How far the offset boundary is from where it should be: {max, avg} over vertices.
     *
     * The absolute error |dist(v, input complex) - target_distance|, taken over the
     * offset-boundary vertices only. The MAX is what the optimization converges against: the
     * offset is only as good as its worst-placed vertex, so an average that looks fine can still
     * hide a stretch of boundary sitting far off the target. Mirrors the 3D
     * TopoOffsetTetMesh::compute_distance_deviation().
     */
    std::pair<double, double> compute_distance_deviation() const;

    /// The vertex compute_distance_deviation() last found the max at, and a dump of everything
    /// that could be stopping it from moving. Diagnostic only.
    mutable size_t m_worst_dist_vid = static_cast<size_t>(-1);
    void log_worst_dist_vertex() const;

    /// Whether this edge is on the band's OUTER surface, recomputed from the tags on every call
    /// -- the live counterpart of edge_is_offset(), for use inside the operation passes.
    bool edge_is_offset_surface_live(const Tuple& e) const;

    /// Seed the sizing field from the offset's CURRENT edge lengths (paper Sec. 5.3.3, Step 1),
    /// once, before the first operation pass. Without it the field starts at the background
    /// target length and the first collapse pass decimates the offset.
    void init_offset_sizing_field();

    /// {max_dist_err, avg_dist_err, max_phi_residual, avg_phi_residual, max_grad, avg_grad,
    /// max_grad_at_vertex, max_grad_in_edge}. max_grad is the convergence criterion -- the
    /// full placement-gradient norm at band vertices, so max_grad_at_vertex repeats it and
    /// max_grad_in_edge is the chord diagnostic. The rest are diagnostics kept because they
    /// answer different questions -- the Euclidean error says how far the smoothed offset
    /// ended up from the exact one, and the Phi residual still ranks the sizing field. One
    /// entry for the whole run, as in 3D.
    std::vector<std::array<double, 8>> optimization_metrics;
    /// {split-born vertices, recollapsed, recollapsed in the immediately following collapse
    /// pass} per A/B round, in step with op_counts. See VertexExtra2d::m_born_epoch.
    std::vector<std::array<int, 3>> churn_counts;
    /// {splits, collapses, swaps} per A/B ROUND -- one entry per round the driver runs,
    /// including the round that converges, as deltas rather than running totals. Phase B does no
    /// topological work, so a round's entry is exactly what its Phase A did. NOTE this does NOT
    /// mirror optimization_metrics, which is a single whole-run summary.
    std::vector<std::array<int, 3>> op_counts;
    /// Whether the optimization met the convergence criterion before the round cap.
    /// The A/B round the run is in, 1-based; 0 before the loop starts. Read only by
    /// write_smoothing_debug_output(), to tag each frame with the sub-iteration it belongs to.
    int m_ab_round = 0;
    /// Monotonic frame counter for the debug timeline. Mutable because the write hook is const.
    mutable size_t m_debug_seq = 0;
    /// Pass index within the current phase, and the (round, phase) it belongs to -- when those
    /// change the index restarts. All three exist only to name frames.
    mutable int m_debug_pass = 0;
    mutable int m_debug_last_round = -1;
    mutable char m_debug_last_phase = '?';
    /// See offset_gradient_tolerance(). Set once by measure_gradient_reference().
    double m_gradient_reference = 0.;
    bool m_converged = false;

    /// CHURN: split-born vertices that a collapse later removed, and the subset removed in the
    /// same pass-pair that created them.
    std::atomic<int> iter_cnt_split_born{0};
    std::atomic<int> iter_cnt_recollapsed{0};
    std::atomic<int> iter_cnt_recollapsed_same_pass{0};
    std::atomic<int> iter_cnt_split = 0, iter_cnt_collapse = 0, iter_cnt_swap = 0;
    std::atomic<int> iter_cnt_collapse_offset_removed{0};
    /// Operations refused because they would have left an offset-boundary face over tolerance.
    std::atomic<int> iter_cnt_collapse_offset_reject{0};
    std::atomic<int> iter_cnt_swap_offset_reject{0};
    /// Splits of an OFFSET-boundary edge: offered, accepted.
    std::atomic<int> iter_cnt_split_offset_before{0};
    std::atomic<int> iter_cnt_split_offset{0};
    /// Dispatch: the optimization phase runs the shared split, everything else the
    /// marching-triangles one.
    /// Parent face labels for an optimization split, keyed by the apex vertex opposite the
    /// split edge -- that vertex is shared by both children of the same parent, so it names
    /// them afterwards. Keyed and consumed EXACTLY as TriOptimizerMesh::split_edge_after keys
    /// and consumes its own FaceAttributes cache, so the label lands wherever the tags do.
    /// The endpoints are recorded because the consumer needs the two new edges to resolve
    /// each child's apex, and split_after_vertex() is handed only the new vertex.
    struct OptSplitCache2d
    {
        std::map<size_t, int> face_label;
        size_t v1_id = 0;
        size_t v2_id = 0;
        /// The endpoints' mask AND, captured before the split (3D's rule at both of its split
        /// sites). Consumed by split_after_vertex() behind the parent edge's own class gate,
        /// which is what keeps a chord's midpoint maskless -- so the AND cannot over-claim
        /// through one. Never derived from the incident faces' current tags: the band retag
        /// empties the live symmetric difference on every region edge it swallows.
        uint64_t edge_bits = 0;
        /// Diagnostic: the two parent faces' AMIPS before the split, so split_after_vertex() can
        /// say whether a needle child came from a healthy parent (the split MADE it) or from a
        /// parent that was already unscoreable (the split only copied it).
        double parent_q_max = -1.;
        /// Same question in the scale-invariant measure, which keeps resolving after AMIPS has
        /// saturated at MAX_ENERGY. MIN over the parents: the flattest thing the split inherited.
        double parent_flatness = 1.;
    };
    wmtk::threading::enumerable_thread_specific<OptSplitCache2d> m_opt_split_cache;

    bool marching_split_edge_before(const Tuple& t);
    bool marching_split_edge_after(const Tuple& t);

    /**
     * @brief Reject any collapse that violates the substructure link condition.
     *
     * The base applies it only when BOTH endpoints already sit on a tracked surface or the
     * bbox, which is the right rule for tetwild and simwild. It is not enough here: the offset
     * region is a thin band, and a collapse with only one endpoint on the boundary can still
     * pinch the two sides of that band together, which is precisely what makes the region stop
     * being manifold. The offset asks unconditionally.
     */
    bool collapse_edge_before(const Tuple& t) override;

    /**
     * @brief Reject a flip whose new edge already exists.
     *
     * Flipping (a,b) to (c,d) when c and d are already joined creates a second edge between the
     * same pair of vertices. Across a thin offset band that is exactly how the two sides of the
     * band get stitched together, and the region stops being manifold. The base refuses
     * tracked-surface edges but has no reason to check this.
     */
    bool swap_edge_before(const Tuple& t) override;

    /// Only to count an accepted flip. The base decides whether it is accepted; there is nothing
    /// the offset needs to do to a flip that survives swap_edge_before().
    bool swap_edge_after(const Tuple& t) override;
    bool collapse_before_vertex(size_t v1, size_t v2) override;
    void collapse_after_vertex(size_t v1, size_t v2) override;
    void split_after_vertex(size_t v_new) override;

    /**
     * @brief Carry each parent's region label onto the two children it became.
     *
     * This is bookkeeping, not positioning, and split_after_vertex() would be the natural home
     * for it. It lives here because of WHEN the base calls the two. Everything downstream of
     * the label reads it during the same split: the containment check inside the base's
     * split_edge_after() runs surface_segment_is_outside() on both new segments, which reaches
     * surface_envelope_for_edge() and so the endpoints' boundary masks. split_after_vertex()
     * runs after that check; this hook is the last one the base offers before it.
     *
     * Getting it wrong is silent in the dangerous direction: children still holding whatever
     * occupied their recycled fid slots classify as "not a region boundary", which yields a
     * null envelope, and a null envelope makes surface_segment_is_outside() return FALSE --
     * containment skipped rather than failed. Measured on the dragon rectangle, that decayed
     * the offset polyline 429 -> 86 edges and took max_dist_err from 0.000215 to 0.000302.
     *
     * Position is left entirely to the base; this always returns its result unchanged.
     */
    bool split_adjust_position(size_t v_new, const std::vector<Tuple>& children) override;

    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    /**
     * @brief IDENTIFICATION ONLY -- no operation refuses the domain wall through these.
     *
     * The wall is a tracked region boundary like every other one: init_surfaces_and_boundaries()
     * tags its edges m_is_surface_fs, masks its vertices with ambient's bit, and puts its
     * segments in ambient's envelope, so refinement, coarsening, flips and smoothing are all
     * governed by the same envelope containment, per-class merge rules and link conditions that
     * govern the input complex. The categorical refusals that used to sit in the split, swap
     * and collapse hooks are gone (2026-08-22): they duplicated -- and where they differed,
     * contradicted -- that accounting, and 3D's hooks never had them.
     *
     * What still reads these two:
     *  - band_vertex_is_reachable(): a wall-clipped offset vertex is booked PINNED for the
     *    convergence criterion, since Phase B's held-gate skips it and gating on it would
     *    deadlock the run.
     *  - the base's own wall rules, which stand apart from the component: the collapse
     *    on_bbox_faces subset rule (wall vertices merge only along their own wall;
     *    TriOptimizerMeshCollapse.cpp) and the smoothing wall freeze -- which the component's
     *    smooth_before() deliberately BYPASSES in favour of envelope containment.
     *  - diagnostics: the tracked-edge census, the mask-health class breakdown, and
     *    warn_if_offset_reaches_domain_boundary().
     */
    bool vertex_is_on_domain_boundary(const size_t vid) const
    {
        return !m_vertex_attribute[vid].on_bbox_faces.empty();
    }
    bool edge_is_on_domain_boundary(const size_t eid) const
    {
        return m_edge_attribute[eid].m_is_bbox_fs >= 0;
    }

    /**
     * @brief Classify every region boundary, build the per-tag containment envelopes, and tag
     * the domain wall -- once, from the INPUT mesh, before offset construction runs.
     *
     * The 2D twin of TopoOffsetTetMesh::init_surfaces_and_boundaries(), called from the same
     * place (init_from_image) for the same reason: the band's tags REPLACE a face's own rather
     * than joining them, so a region the band grows through loses its tag there and its boundary
     * curve is truncated at the band. An envelope built afterwards is a tube around the
     * truncated curve. Built here, each E_t follows the region's original curve.
     *
     * A region boundary is an edge whose two incident faces carry different tag sets; the edge
     * enters the bucket of every tag on exactly one side (the symmetric difference). An edge
     * with only ONE incident face is the domain wall -- the boundary against the unmeshed
     * outside -- and enters the buckets of its single face's tags, which is how ambient's
     * envelope comes to hold the box.
     *
     * Requires the face tags to be set, which init_from_image() does just above the call.
     */
    void init_surfaces_and_boundaries();

    /// Set VertexExtra2d::m_is_on_input from the construction labels, once label_input_complex()
    /// has evaluated the selection. Separate from init_surfaces_and_boundaries(), which runs
    /// earlier and can only see tag boundaries. The 3D twin is mark_input_complex_vertices().
    void mark_input_complex_vertices();

    /**
     * @brief Warn if the offset band has grown into the domain boundary.
     *
     * When target_distance exceeds the clearance between the input complex and the bounding
     * box, construction runs out of room and the band's outer boundary becomes the box
     * itself. Two things then go wrong at once, and neither is visible in the report:
     *
     *  - Those vertices are ON the bbox, which is categorically frozen, so no split, collapse,
     *    swap or smooth may move them. The target distance is unreachable there by construction.
     *  - compute_distance_deviation() cannot even see them. It walks edges and skips any with
     *    no opposite face, which is exactly what a band edge lying on the domain boundary is,
     *    so the clipped stretch enters neither max_dist_err nor avg_dist_err.
     *
     * Measured on the dragon rectangle at target_distance 0.1 (domain 1.07 x 0.98): 101 band
     * vertices pinned on the box, true error there up to 0.0516 -- 52% of target_distance --
     * while the report showed max_dist_err 0.00477, under-reporting by 11x. The run looks like
     * a near-miss and is actually a structural failure, which is why this is a warning and not
     * a debug line.
     */
    void warn_if_offset_reaches_domain_boundary() const;

    /**
     * @brief What smoothing did with each class of vertex, per pass.
     *
     * The base's SmoothRejectCounters says WHY a move was refused; it cannot say what kind of
     * vertex was asking. Every vertex now takes the same path -- the shared smoother -- so what
     * is worth counting here is only the dispatch: how many attempts were on the offset
     * boundary (the ones carrying the offset term), how many on another region's boundary, and
     * how many were turned away before the smoother saw them at all.
     *
     * The old per-site offset counters are gone with the placement they instrumented: there is
     * no separate offset path left to instrument, and "clamped", "slid" and "on the complex"
     * were properties of a hand-rolled projection that no longer exists.
     */
    struct SmoothTrace
    {
        std::atomic<int> attempted{0}; ///< smooth_before() entered
        std::atomic<int> before_bbox{0}; ///< base smooth_before said no: on the bounding box
        std::atomic<int> before_unrounded{0}; ///< base smooth_before said no: could not round
        std::atomic<int> before_phase_b_not_offset{0}; ///< Phase B: wrong class for this sub-sweep
        std::atomic<int> before_phase_b_enveloped_background{0}; ///< Phase B: envelope-held
        std::atomic<int> before_phase_b_enveloped_offset{0}; ///< Phase B: on-offset AND held
        std::atomic<int> offset_attempted{0}; ///< reached the smoother with the offset term
        std::atomic<int> offset_accepted{0}; ///< ... and the smoother kept the new position
        std::atomic<int> interior_attempted{0}; ///< reached it without one
        std::atomic<int> region_attempted{0}; ///< ... of which sat on another region's boundary
        /// Phi residual over the offset vertices this pass actually touched, before and after,
        /// in units of 1e-9 so an integer atomic can accumulate a sum and a max.
        ///
        /// This is the one number that says whether the offset term is DOING anything. An
        /// acceptance count cannot: the shared smoother's quality veto refuses a move outright
        /// rather than shortening it, so a pass can accept most of what it tries and still
        /// leave every badly-placed vertex exactly where it was.
        std::atomic<long long> res_before_nano{0};
        std::atomic<long long> res_after_nano{0};
        std::atomic<long long> res_max_before_nano{0};
        std::atomic<long long> res_max_after_nano{0};

        void reset()
        {
            for (std::atomic<int>* c :
                 {&attempted,
                  &before_bbox,
                  &before_unrounded,
                  &before_phase_b_not_offset,
                  &before_phase_b_enveloped_background,
                  &before_phase_b_enveloped_offset,
                  &offset_attempted,
                  &offset_accepted,
                  &interior_attempted,
                  &region_attempted}) {
                c->store(0);
            }
            for (std::atomic<long long>* c :
                 {&res_before_nano, &res_after_nano, &res_max_before_nano, &res_max_after_nano}) {
                c->store(0);
            }
        }
    };
    SmoothTrace m_smooth_trace;

    /**
     * @brief Why smoothing does not lift a sliver's apex off its opposite edge.
     *
     * Interleaved smoothing IS on (interleaved_smoothing defaults true), so every needle-adjacent
     * vertex is visited after every topological pass. These counters say what happens when it is:
     *
     *  - offered   : smooth_before() entered with a needle already in the one-ring
     *  - reached   : the solve produced a candidate and smooth_after() saw it
     *  - fixed     : that candidate actually dropped the ring's worst below kNeedleQuality
     *  - stationary: the candidate moved the vertex less than 1e-12 -- the solve found nothing
     *
     * `offered` minus `reached` is the search failing outright; `reached` minus `fixed` is a move
     * being made that does not repair the sliver. The two have different causes and the fix for
     * one is not the fix for the other, which is why they are counted separately.
     */
    mutable wmtk::threading::enumerable_thread_specific<std::pair<double, Vector2d>> m_needle_pre;
    mutable std::atomic<size_t> m_needle_smooth_offered{0};
    mutable std::atomic<size_t> m_needle_smooth_reached{0};
    mutable std::atomic<size_t> m_needle_smooth_fixed{0};
    mutable std::atomic<size_t> m_needle_smooth_stationary{0};
    /// Worst-case record: the best (lowest) ring max any needle-adjacent smooth achieved.
    mutable std::atomic<size_t> m_needle_smooth_reports{0};

    /// Max AMIPS over the faces incident to `vid`. -1 if it has none.
    double ring_max_quality(size_t vid) const;

    /**
     * @brief Scale-invariant flatness: 2*area / longest_edge^2.
     *
     * ~0.433 for an equilateral triangle, -> 0 as the three vertices become collinear, and
     * independent of size -- which AMIPS is too, but AMIPS saturates at the MAX_ENERGY sentinel
     * while this keeps resolving all the way down. It is the measure the genesis tracking needs:
     * "this face got flatter" is a statement AMIPS cannot make once it is unscoreable.
     */
    double face_flatness(size_t fid) const;

    /**
     * @brief The full post-mortem on why nothing removes the flat faces.
     *
     * For the worst faces by flatness, enumerates all three edges and reports, per edge, every
     * gate that decides whether an operation may touch it:
     *   - length vs the COLLAPSE gate (4/5 l s-bar) -- above it, the edge never reaches the
     *     candidate list, so collapse_edge_before is never called
     *   - length vs the SPLIT gate (4/3 l s-bar), and whether it is force-split queued
     *   - is_edge_on_surface -- swap_weight returns lowest() for a surface edge, so it is never
     *     swapped at all
     *   - swap_weight -- the pass needs > 1e-5, i.e. a strict improvement of the pair's worst
     * Plus a scan for COINCIDENT VERTICES (pairs closer than 1e-9 * l), with whether each pair
     * shares an edge: a pair that does not is geometry no local operation can reach.
     */
    void needle_forensics() const;

    /// Genesis: flatness transitions recorded at the operation hooks. {op, parent, child}.
    void record_flatness(const char* op, double parent_flat, size_t child_fid) const;
    mutable std::atomic<size_t> m_flat_created_split{0};
    mutable std::atomic<size_t> m_flat_created_collapse{0};
    mutable std::atomic<size_t> m_flat_worsened_split{0};
    mutable std::atomic<size_t> m_flat_genesis_reports{0};
    static constexpr double kFlatThreshold = 1e-3;
    /// The flattest face in the collapse's ring before it ran, for record_flatness().
    mutable wmtk::threading::enumerable_thread_specific<double> m_collapse_parent_flatness;
    void log_smooth_trace() const;

    /**
     * @brief WHERE A PHASE B OFFSET PLACEMENT STOPPED, and therefore why the vertex did or did
     *        not move.
     *
     * smooth_offset_vertex_backtracking() has a dozen exits and only three of them mean "this
     * vertex is where it wants to be". The counters that existed before could not tell them apart:
     * `constrained` lumps the two ring refusals together, and everything else -- including the
     * paths that put the vertex back exactly where it started and return true -- was reported as
     * `placed`. A vertex that never took a step looked identical in the log to one that walked
     * onto the level set, which is exactly the question "some vertices are not moving" asks.
     *
     * The zero-entry-gradient cases are the ones worth separating. q = ||2 (Phi - c) grad
     * Phi|| is zero when Phi == c (done) or when grad Phi == 0 (a pinch minimum, a legitimate
     * stationary point of E), and negative where the field came back non-finite. Only the
     * first is convergence.
     */
    enum class PlacementStop : int {
        Moved = 0, ///< took >= 1 step and got under the run's own gradient bar
        IterCap, ///< moved, but ran out of kMaxDescentIters (2 delta of travel)
        RingBlocked, ///< the FIRST trial step inverted the one ring: displacement is exactly 0
        LeftRing, ///< left the ring after >= 1 accepted step; kept the last inside position
        MidStationary, ///< grad E hit zero part-way down the walk
        MidNonFinite, ///< Phi or grad Phi went non-finite part-way down the walk
        PreInverted, ///< a one-ring face was already inverted in floats on entry
        NoRing, ///< empty one-ring
        OnLevelSet, ///< q(entry) == 0 because Phi == c. Clean "nothing to do".
        UnderBar, ///< q(entry) already <= the run's convergence bar; equally clean
        Stationary, ///< q(entry) == 0 because grad Phi == 0 (pinch minimum)
        NonFinite, ///< Phi, grad Phi, or the vertex position itself non-finite at entry
        LeftOffset, ///< the step would have left the closed offset region {Phi >= c}; refused
        EnvelopeBlocked, ///< no tangent could be resolved on the vertex's own boundary curve
        ChordBlocked, ///< sliding any distance put an incident region edge outside its tube
        /// PRESSED: every trial step that kept the ring valid put an incident face over
        /// stop_energy (or made one already over it worse). The vertex is held by the MESH's
        /// quality bar, not by the field -- the two-fronts seam. See smooth_offset_vertex_backtracking.
        QualityBound,
        COUNT
    };
    static const char* placement_stop_name(PlacementStop s);

    struct PlacementTrace
    {
        static constexpr size_t N = size_t(PlacementStop::COUNT);

        /// One vertex, kept per reason so a count can be turned into something to go and look
        /// at. The one kept is the worst: largest |Phi - c|, i.e. the furthest off the level set.
        struct Exemplar
        {
            size_t vid = size_t(-1);
            double res = 0.; ///< Phi - c at entry; 0 where it was never measured
            double gnorm = 0.; ///< |grad Phi| at entry
            double cosn = 0.; ///< |grad Phi . n| / (|grad Phi| |n|) -- 0 is exactly tangent
            double nnorm = 0.; ///< |n|; 0 means offset_surface_normal() found nothing
            double disp = 0.; ///< how far the vertex actually moved this visit
            int iters = 0; ///< accepted descent steps
            double x = 0., y = 0.; ///< where it is, so it can be found in the viewer
        };

        std::array<std::atomic<int>, N> n{};
        std::array<std::atomic<int>, N> n_reachable{}; ///< ... of which gate convergence
        std::atomic<int> moved_one_step{0}; ///< Moved, but with exactly one step
        std::mutex mtx;
        std::array<Exemplar, N> worst{};

        void reset()
        {
            for (size_t i = 0; i < N; ++i) {
                n[i].store(0);
                n_reachable[i].store(0);
                worst[i] = Exemplar{};
            }
            moved_one_step.store(0);
        }

        void record(PlacementStop s, bool reachable, const Exemplar& e)
        {
            const size_t i = size_t(s);
            ++n[i];
            if (reachable) ++n_reachable[i];
            // Racy pre-check, then the real one under the lock: this runs once per offset vertex
            // per pass and the exemplar is a diagnostic, so the cost matters more than which of
            // two equally bad vertices wins a tie. The empty test comes first so that every
            // non-zero count is guaranteed an exemplar, including the reasons whose residual is
            // never measured (an inverted ring is refused before Phi is ever evaluated).
            const bool empty = worst[i].vid == size_t(-1);
            if (!empty && std::abs(e.res) <= std::abs(worst[i].res)) return;
            std::lock_guard<std::mutex> lock(mtx);
            if (worst[i].vid == size_t(-1) || std::abs(e.res) > std::abs(worst[i].res)) {
                worst[i] = e;
            }
        }
    };
    PlacementTrace m_placement_trace;
    void log_placement_trace() const;

    /**
     * @brief AUDIT THE PHASE B CONTRACT: no offset-boundary vertex may be envelope-held there.
     *
     * Phase B releases the offset envelope and keeps the region envelopes, so a vertex that is
     * ONLY on the offset is free and a vertex that is also on a region boundary is held -- and
     * smooth_before() skips the held ones outright, because placing one needs the envelope's
     * pull and containment together with the offset term. On a model whose offset never
     * coincides with a region boundary that set must be EMPTY.
     *
     * The audit is not the same test as the dispatch: it re-derives "on a region boundary" from
     * the vertex's own incident edges (edge_is_region), which is ground truth, and compares it
     * against the stored m_is_on_region / on_bbox_faces the dispatch reads. A vertex held by the
     * stored flag with no live region edge under it is a STALE flag -- the offset vertex is being
     * frozen by bookkeeping rather than by geometry.
     */
    void audit_phase_b_offset_envelope_holds() const;

    /**
     * @brief Are the tracked region boundaries actually CONTAINED by anything?
     *
     * A class-0 edge is dispatched to an envelope by its boundary MASK, and the mask is the
     * symmetric difference of its two faces' tags. An edge whose faces carry the SAME tags has an
     * empty difference, so envelope_for_mask() gives it nullptr: it is tracked as a region
     * boundary and held by nothing, in either phase. Construction cannot produce one -- it skips
     * same-tag edges before it ever marks them -- so a non-zero count here is something that
     * happened afterwards, and it is a hole in the containment the per-tag envelopes exist to
     * provide. Called at construction and at each Phase B entry so the two can be compared.
     */
    void log_region_edge_mask_health(const std::string& when) const;

    /**
     * @brief WHICH tracked edges are outside their envelope, and by how much.
     *
     * The shared pass driver's sanity_checks() already reports "Edge [a, b] is outside!" for any
     * m_is_surface_fs edge that fails surface_segment_is_outside(). That message says nothing
     * about WHICH envelope refused it, and the answer forks the diagnosis completely:
     *
     *   OFFSET-CLASS (mask 0, both endpoints on the offset)  -- refused by m_offset_envelope, the
     *     Phase A pin at ab_offset_envelope_rel x target_distance. Means Phase A moved the offset
     *     boundary out of the tube that is supposed to hold it where Phase B left it.
     *   REGION-CLASS (mask != 0) -- refused by that tag's tube, or by the INTERSECTION of several
     *     tubes at a junction. Means a tag-region boundary has drifted off the input partition,
     *     which is a containment failure of the thing the envelopes exist to protect.
     *
     * Reports per-endpoint distance to each REAL member tube. A multi-bit mask dispatches an
     * IntersectionEnvelope, which must never be asked squared_distance (TagEnvelopes.hpp: its
     * BVH is null), so the members are walked individually instead of querying the composite.
     *
     * Call it at construction as well as inside the loop: an edge already outside BEFORE any
     * operation runs is a construction defect, and that is a different bug from one an operation
     * created. Diagnostic only -- reads the mesh, writes only the log.
     */
    void audit_surface_containment(const std::string& when) const;

    /// How many offset placements this pass could not take to their own minimum -- the visit
    /// entered its one-ring bisection, was refused outright by inversion, or found the ring
    /// already float-inverted on entry. Reset by phase_b_smooth() before each pass; a pass that
    /// ends with this at zero is the loop's natural exit.
    mutable std::atomic<int> m_phase_b_constrained{0};

    /// How many offset placements ran under the DEFAULT (normal-projected gradient) term but had
    /// no usable n_i, so the term fell back to the plain quadratic error. Never reset -- it is a
    /// run total, reported at the end of each Phase B. The invariant is 0; a nonzero count means
    /// offset_vertex_normal() is returning the zero vector for real vertices, which is worth
    /// knowing before reading the placement numbers.
    mutable std::atomic<int> m_placement_no_normal{0};

    /// How many Phase B offset placements found the vertex ALREADY outside its own envelope on
    /// entry. Since 2026-08-23 this disables nothing -- the post-step projection pulls such a
    /// vertex back in -- but it is still the number to read first: the invariant is 0, and a
    /// nonzero count means construction or Phase A is leaving offset vertices outside their
    /// region tube. Never reset -- a run total.
    mutable std::atomic<int> m_placement_env_entry_outside{0};

    /// How many Phase B offset placements had their accepted step PROJECTED back into the
    /// vertex's region tubes. The expected steady state on any multi-tag model whose offset
    /// coincides with a region boundary: it counts the vertices doing constrained motion along a
    /// boundary curve, not a problem. Read it against the EnvelopeBlocked count in the
    /// placement-stop breakdown, which is where a projection that could NOT be committed lands.
    /// Never reset -- a run total.
    mutable std::atomic<int> m_placement_projected{0};

    /// How many Phase B offset placements were solved TANGENTIALLY -- reduced to arclength along
    /// the vertex's own tag boundary curve, rather than stepped freely in 2D. The expected
    /// steady state for every offset vertex that a region envelope also holds. Read it against
    /// ChordBlocked in the placement-stop breakdown, which counts the visits whose slide was cut
    /// off by an incident chord leaving its tube -- i.e. the mesh wanting a vertex at a corner.
    /// Never reset -- a run total.
    mutable std::atomic<int> m_placement_tangential{0};

    /// Per-thread Newton solver for Phase B's interior AMIPS solves. Separate from the base's
    /// m_solver because it carries a different stopping rule: polysolve's rel_grad_norm_tol set
    /// to ab_vertex_grad_tol_rel with a deep iteration budget, against the base's fixed shallow
    /// budget with no tolerance. Created on first use.
    mutable wmtk::threading::enumerable_thread_specific<
        std::unique_ptr<polysolve::nonlinear::Solver>>
        m_phase_b_solver;

    ////// wmtk::TriOptimizerMesh hooks

    /**
     * @brief Is this vertex on a region boundary -- a tag boundary, or the domain wall.
     *
     * DERIVED, not stored, exactly as in 3D. Both halves are already maintained: m_is_on_region
     * by the split/collapse hooks, on_bbox_faces by set_intersection of the split endpoints and
     * by the collapse rule that a wall vertex may only merge into one at least as constrained.
     */
    bool vertex_is_on_region(const size_t vid) const
    {
        return m_vertex_extra[vid].m_is_on_region || !m_vertex_attribute[vid].on_bbox_faces.empty();
    }

    /// The three helpers of the per-tag envelope dispatch. tag_bits() and edge_mask() are
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
     * splitting precisely such edges. vertex_is_on_region() is the predicate that does NOT
     * over-claim, so the mask says WHICH boundaries and this says whether the vertex is on one
     * at all; the mask only ever narrows an answer the flags already allow.
     */
    uint64_t vertex_boundary_mask(const size_t vid) const
    {
        return vertex_is_on_region(vid) ? m_vertex_extra[vid].m_boundary_mask : uint64_t(0);
    }

    /// A segment lies on a boundary only if BOTH ends do: the AND of its endpoints' masks. The
    /// 2D twin of face_mask(), which ANDs three.
    uint64_t edge_mask(const std::array<size_t, 2>& vids) const
    {
        return vertex_boundary_mask(vids[0]) & vertex_boundary_mask(vids[1]);
    }

    /**
     * @brief DIAGNOSTIC ONLY: which tag boundaries the incident faces say this edge lies on,
     * RIGHT NOW -- the same symmetric difference init_surfaces_and_boundaries() classified by.
     *
     * Nothing dispatches or propagates from this any more. It is only trustworthy while the
     * face tags are still the input's own: execute_offset() replaces the tags of every face the
     * band grows through, after which this is empty across every region edge the band swallowed
     * -- which is exactly how deriving split masks from it minted uncontained region vertices
     * (see log_region_edge_mask_health, which counts the divergence between this and the stored
     * masks). New vertices take the ENDPOINTS' mask AND behind the parent edge's class gate --
     * 3D's rule at both of its split sites; the gate is what keeps the historical chord
     * over-claim (24 marching half-edges held to a tube a full target_distance away, on
     * topo_annots_groups) from returning, because a chord is not a region-class edge and never
     * reaches the AND.
     */
    uint64_t edge_boundary_bits(const Tuple& e) const
    {
        const std::optional<Tuple> opp = e.switch_face(*this);
        if (!opp) {
            return tag_bits(m_face_attribute[e.fid(*this)].tags); // domain wall
        }
        const auto& t0 = m_face_attribute[e.fid(*this)].tags;
        const auto& t1 = m_face_attribute[opp->fid(*this)].tags;
        CellTag diff;
        std::set_symmetric_difference(
            t0.begin(),
            t0.end(),
            t1.begin(),
            t1.end(),
            std::inserter(diff, diff.begin()));
        return tag_bits(diff);
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

    /**
     * @brief Class-0 segments -- every region boundary, the input complex and the domain wall
     * included -- carry a containment requirement; the offset boundary does not.
     *
     * The envelope holds the other tag regions where they are, and -- since the complex stopped
     * being frozen -- the input complex too. That half is exactly TriWild's input envelope: the
     * complex may be split, collapsed and smoothed, and this is what bounds how far the result
     * may drift from the geometry as loaded.
     *
     * The offset boundary is exempt IN PHASE B, where it is the surface the optimization exists
     * to move and containing it inside a tube around its initial position would cap how far it
     * can ever travel. In PHASE A it is held by m_offset_envelope instead -- a tube of one Phi
     * tolerance around wherever Phase B last left it, rebuilt each round. That is what turns "do
     * not degrade the offset" from a per-operation criterion into a geometric constraint every
     * shared operation already honours through surface_edge_is_outside().
     *
     * Null means "no containment requirement", which the base handles by skipping the check.
     */
    std::shared_ptr<SampleEnvelope> surface_envelope_for_edge(
        const std::array<size_t, 2>& vids) const override
    {
        // BOUNDARY GEOMETRY FIRST, in BOTH phases: a segment on any tag-region boundary may not
        // drift out of that boundary's tube, and a segment on several boundaries -- a junction
        // -- is held in their intersection. The mask carries the input complex too: every
        // complex simplex lies on tag boundaries (label_input_complex() can only label an
        // isolated simplex whose face star is tag-heterogeneous), so the per-tag tubes subsume
        // the deleted single region-boundary envelope, as-loaded geometry and all -- E_t is
        // built from the INPUT mesh before construction touches it.
        //
        // KEYED ON THE VERTICES, which is why no `live` recomputation from face tags is needed
        // any more. Every caller is an operation asking about a segment it is about to create
        // or has just created, whose own edge attributes are not written yet; the endpoints'
        // masks are, and they are maintained by the operations themselves (AND at a split, OR
        // at a collapse). That is what edge_is_region_boundary_live() existed to work around.
        uint64_t mask = edge_mask(vids);
        bool all_offset = true;
        for (const size_t v : vids) {
            all_offset = all_offset && m_vertex_extra[v].m_is_on_offset;
        }

        // THE AMBIGUOUS CASE, AND WHERE THE "Edge [V, V] is outside!" ERRORS CAME FROM. The two
        // endpoints can each be on region boundaries AND on the offset front -- that is a real
        // state, not a defect, and the whole point of tracking the two families separately. But
        // then the endpoint-mask AND is only a NECESSARY condition for the SEGMENT lying on a
        // shared boundary, never a sufficient one: two vertices on different junctions can share
        // a tag bit by coincidence and be joined by an offset chord that lies nowhere near that
        // tag's curve. Measured on topo_annots_groups (tag_0 & tag_2, delta 1.2): edge
        // [2624, 2700], own class OFFSET, endpoint masks 0x6 and 0xc, AND 0x4 -- held to tag_2's
        // tube, whose half-width is 0.0707, with its midpoint 0.25 outside it. One such edge at
        // construction, 21 by the end of Phase A, every one of them refused refinement by a
        // container it was never meant to satisfy.
        //
        // So when the endpoints claim both families, ASK THE EDGE'S OWN CLASS, which is the only
        // record that distinguishes a chord from a boundary.
        //
        // WHY READING THE SLOT IS SAFE HERE, and would not be unconditionally. The base checks a
        // SPLIT's two child segments (TriOptimizerMeshSplit.cpp:297, :300) BEFORE writing their
        // attributes at :305, so their slots still hold a recycled edge's class -- reading that
        // is the same hazard split_adjust_position() exists to avoid on the vertex side. It
        // cannot bite here, because a split child never reaches this branch: the child's mask
        // and its offset flag are already mutually exclusive by construction. A parent of class
        // 0 gives the midpoint m_is_on_region = true and m_is_on_offset = false; an offset
        // parent gives the reverse and, with m_is_on_region false, vertex_boundary_mask() zeroes
        // the midpoint's mask outright. Either way one of `mask` and `all_offset` is empty and
        // the test below is never entered. The m_is_surface_fs guard is the belt to that
        // braces: an illegible slot leaves both constraints standing rather than dropping one.
        if (mask != 0 && all_offset) {
            if (const auto found = try_tuple_from_edge(vids)) {
                const size_t eid = std::get<1>(*found);
                if (m_edge_attribute[eid].m_is_surface_fs) {
                    if (edge_is_offset(eid)) {
                        mask = 0; // an offset edge lies on no region boundary
                    } else {
                        all_offset = false; // a region edge is not the offset front
                    }
                }
            }
        }
        // Both families compose: whatever holds this segment holds it at once. Phase A holds the
        // offset where Phase B left it; Phase B is what moves it, so it contributes nothing
        // there -- and null when there is no offset envelope yet, which is the pre-pass, before
        // the offset exists at all.
        return containment_for(mask, all_offset);
    }

    /**
     * @brief NO per-vertex positional constraint. The per-tag envelopes closed the hole this
     * filled, structurally -- the same deletion 3D made to its lower-strata point refusal.
     *
     * This used to pin an input-complex vertex to within envelope_size of the complex by BVH
     * distance, because the single fused region envelope was built from SEGMENTS and an input
     * made of ISOLATED POINTS contributed none: nothing contained those vertices, they drifted,
     * the band followed, and its boundary ended up 0.001 from the original point -- where Phi
     * diverges and the run's convergence metric is meaningless. Measured on
     * topological_offset_2d_vertex_input.
     *
     * An isolated point of the complex only ever arises where two or more selected tags meet
     * (see label_input_complex(): the boolean selection can only label an isolated simplex whose
     * face star is tag-heterogeneous), so the edges radiating from it are tag boundaries and its
     * boundary mask carries several bits. smoothing_containment_envelope() therefore hands the
     * smoother an IntersectionEnvelope -- within eps of EVERY curve it lies on -- which pins it
     * to the junction more tightly than this ball ever did, and the pull toward the
     * most-violated member drags it back if it strays. The base's hook is pure virtual, so this
     * stays as the honest constant rather than being deleted outright.
     */
    bool smoothing_position_is_allowed(const size_t, const Vector2d&) const override
    {
        return true;
    }

    /**
     * @brief The offset boundary is the one tracked surface with NO envelope, in either role.
     *
     * It is the surface the optimization exists to move: a tube around wherever construction
     * left it would cap how far it can ever travel toward the level set. What holds it is the
     * offset term in the OBJECTIVE -- the local root find in smooth_offset_vertex_backtracking()
     * -- not a container.
     *
     * THE PULL MUST BE A REAL ENVELOPE, NEVER A COMPOSITE. This hook's consumers call the
     * NON-virtual SampleEnvelope queries -- nearest_point (projected smoothing) and the
     * ExactDistanceEnergy2D trio -- which on a composite would bind to the base's null BVH. So
     * a junction vertex (several mask bits) is pulled toward its MOST-VIOLATED member tube
     * instead: one real envelope per smoothing attempt, alternating projections toward the
     * junction across passes, while the containment intersection below enforces the full
     * constraint.
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
     * @brief ... and it is not CONTAINED by one either, except in Phase A.
     *
     * Phase A is TriWild and its smoothing minimises AMIPS alone; without a container nothing
     * would stop it relocating the offset boundary for the sake of element shape, which is
     * precisely what Phase B then has to undo. Phase B keeps the original answer -- null --
     * because that is the pass whose whole job is to move the boundary.
     *
     * Boundary geometry is contained in the intersection of its tags' tubes, in BOTH phases --
     * the caller only asks is_outside(segment), which composites answer, so unlike the pull
     * this side may hand out an IntersectionEnvelope.
     */
    std::shared_ptr<SampleEnvelope> smoothing_containment_envelope(const size_t vid) const override
    {
        // BOTH FAMILIES, COMPOSED -- not a choice between them. A vertex where the offset front
        // meets one or more region boundaries is on all of them at once, and until 2026-08-23
        // this returned the region tubes ALONE for such a vertex, silently dropping the Phase A
        // offset envelope that is the only thing stopping TriWild from relocating the boundary
        // for the sake of element shape. The three cases now fall out of one expression:
        // pure-offset (mask 0) gives the offset tube in Phase A and null in Phase B, pure-region
        // gives its tubes' intersection in both, and a junction of the two gives the
        // intersection of everything.
        return containment_for(vertex_boundary_mask(vid), m_vertex_extra[vid].m_is_on_offset);
    }

    // NO smoothing_extra_energy OVERRIDE. The base's nullptr is correct for both phases now.
    //
    // This used to hand Phase B an OffsetEnergy2D so the offset term was minimised by the shared
    // AMIPS solver. Phase B no longer uses that solver for the offset at all: offset-only
    // vertices go through smooth_offset_vertex_backtracking()'s 1-D root find, and
    // smooth_before() refuses every other vertex in that sub-sweep. Phase A carries no offset
    // term either -- it is TriWild, and the offset boundary is held there by m_offset_envelope,
    // not by an energy. So the hook has no caller left in either phase. OffsetEnergy2D itself
    // survives; gradient_split() still builds one to measure the placement gradient.

    /**
     * @brief The loop's convergence metric, normalized so that 1.0 means "done".
     *
     * The max of the TWO criteria this optimization has to meet, each divided by its own
     * target, so mesh_improvement() stops exactly when both are met:
     *
     *   - max face AMIPS over stop_energy -- TriWild's, via quality_rel()
     *   - max Phi residual over (convergence_gradient_norm_rel / 2) * target_distance, over the REACHABLE band
     *
     * The average returned alongside it is the same expression over the two averages, so both
     * numbers live on the same 1.0 scale. Nothing reads the average; it is logged.
     */
    std::tuple<double, double> optimization_quality_stats() override;

    /**
     * @brief 1.0 in Phase B, where the metric is normalized; the base's stop_energy in Phase A.
     *
     * THE UNITS ARE PART OF "IDENTICAL TO TRIWILD", and getting this wrong is silent. The pair
     * (optimization_quality_stats, optimization_stop_metric) has to be in ONE set of units,
     * because refine_sizing_around_worst() derives its filter from the first and then compares
     * that filter against a per-face score. Phase A ranks by m_face_attribute[].m_quality, which
     * is absolute AMIPS, so its metric and its bar must be absolute too. In 3D, returning the
     * normalized metric here while Phase A ranked by absolute quality put the filter at 100
     * against a worst element of 97: select_worst_cells returned nothing, no sizing was refined,
     * no force-split edge was ever queued, and Phase A sat at bit-identical 91783.4 for all 20
     * of its iterations with the stall detector firing 19 times and doing nothing.
     */
    double optimization_stop_metric() const override
    {
        return m_phase == OptPhase::A ? wmtk::TriOptimizerMesh::optimization_stop_metric() : 1.;
    }

    /// Samples per band edge; see offset_edge_samples(). 0 falls back to a vertex-only
    /// criterion, which is measurably blind to a band too coarse to be the offset.
    int offset_residual_samples() const { return m_offset_params.offset_residual_samples; }

    /// The residual scale, DERIVED FROM THE CRITERION rather than configured beside it.
    ///
    /// grad E = 2 (Phi - c) grad Phi, so on a field with unit slope at the level set the
    /// gradient bound |grad E| <= g is exactly |Phi - c| <= g/2. That is the whole definition:
    /// half the gradient tolerance, in length units. There is no offset_residual_rel any more.
    ///
    /// WHY IT MATTERS BEYOND REPORTING: this feeds the Phase A offset envelope
    /// (ab_offset_envelope_rel x this) and the derived min_edge_length floor, so loosening the
    /// criterion loosens the tube with it. See the 3D twin for the measurement.
    double offset_residual_tolerance() const
    {
        // DERIVED FROM THE TOLERANCE, not from the knob. grad E . n = 2 (Phi - c) (grad Phi . n),
        // so with |grad Phi| = s at the level set a bound g on the normal gradient is the length
        // bound |Phi - c| <= g / (2 s^2). Reading the knob directly was only correct while the
        // tolerance was itself rel x target_distance x s^2; it is now rel x a MEASURED maximum,
        // and this has to follow it.
        const double s = m_offset_potential ? m_offset_potential->level_set_slope() : 1.;
        return std::max(0.5 * offset_gradient_tolerance() / (s * s), 1e-16);
    }

    /**
     * @brief THE CONVERGENCE TOLERANCE: the bound on |grad (Phi - c)^2| at a band vertex.
     *
     * A fraction of target_distance, which is the right unit: grad E = 2 (Phi - c) grad Phi, and
     * grad Phi is dimensionless for a field whose value is a length, so grad E is a length.
     *
     * WHY THIS REPLACED THE RESIDUAL BOUND. The residual is only comparable to target_distance
     * because residual_length() converts Phi's value into a length -- a conversion each
     * potential has to supply, and one that is only unambiguous where the level set is smooth
     * and the closest feature unique. The gradient needs no such conversion: it is the
     * stationarity condition of the objective Phase B minimises, so it is the same test for the
     * exact Euclidean field and for the smooth potential alike. That is what lets a reentrant
     * input be judged by the same number as a convex one.
     *
     * See the 3D twin, TopoOffsetTetMesh::offset_gradient_tolerance(), for the full derivation
     * of the slope normalization below and for what the bound means at a stationary point of
     * Phi, where no bound on the residual is achievable at all.
     */
    double offset_gradient_tolerance() const
    {
        // A FRACTION OF A MEASURED MAXIMUM, not of an analytic estimate.
        //
        // m_gradient_reference is max |2 (Phi - c) grad Phi . n| over the offset-surface
        // VERTICES as CONSTRUCTED -- the normal-aligned placement gradient of the initial
        // surface, taken once before the A/B loop starts. Everything this bar serves -- the
        // convergence criterion and every Phase B local stop -- compares the FULL gradient
        // norm at those same vertices against it. So the criterion reads "reduce every
        // vertex's placement gradient to this fraction of the worst initial misplacement",
        // and the two sides of the comparison come from one measurement pass.
        //
        // This replaces a normalization by target_distance x level_set_slope()^2, which stated
        // the same intent through an analytic estimate of the field's steepness. The measured
        // reference needs no such estimate and no per-field calibration.
        //
        // Zero until measure_gradient_reference() runs, which would make the bound 1e-16 and
        // convergence unreachable rather than free -- see that function.
        return std::max(
            m_offset_params.convergence_gradient_norm_rel * m_gradient_reference,
            1e-16);
    }

    /// max |2 (Phi - c) grad Phi . n| over the initial offset-surface vertices; the scale
    /// offset_gradient_tolerance() is a fraction of. Zero before measure_gradient_reference().
    double gradient_reference() const { return m_gradient_reference; }

    /**
     * @brief Stop the run if any reachable band vertex has left the potential's support.
     *
     * Beyond dhat, Phi is identically zero WITH a zero gradient: the vertex is given no
     * direction back, its residual saturates instead of growing, and the sizing field refines
     * around a vertex nothing can move. There is no recovery from that state and no honest
     * report of it either, so it is a hard error rather than a warning -- if it turns out to
     * fire on real inputs, the answer is a larger offset_dhat_factor, not a quieter log line.
     *
     * Called once per optimization iteration, and once on the band as constructed.
     */
    void check_offset_within_support(const char* when) const;

    // NO optimization_stalled OVERRIDE, and no optimization_iteration_begin ONE.
    //
    // Both existed for the JOINT loop. The stall override applied the base's inequality per
    // criterion because a run could be stuck on one while the other still moved; Phase A has one
    // criterion -- TriWild's -- so the base's own scalar test is the right one, which is what the
    // override already delegated to in Phase A anyway. optimization_iteration_begin() re-derived
    // the tracked surfaces from the face labels every iteration and logged both criteria; the
    // classification is done once at the top of optimize_offset() now and maintained by the
    // operations, exactly as in 3D, and the per-round criteria are logged by the A/B driver.

    /**
     * @brief The band's distance error, split by whether the optimizer can do anything about it.
     *
     * REACHABLE: a band vertex free to be placed at target_distance. PINNED: one that cannot be,
     * whatever the optimizer does -- it lies on the input complex, where the distance is 0 by
     * definition and the envelope keeps it, or on the domain boundary, where construction ran
     * out of room. Neither is an optimization failure and neither can be improved by
     * refining around it, so only the reachable half drives the loop and the sizing field.
     *
     * They are still reported. compute_distance_deviation() deliberately measures the whole band
     * -- dropping the clipped vertices once let a run whose band was half missing report a 4.8%
     * error and "converge" having measured nothing -- so the two numbers are logged side by side
     * and a pinned vertex out of band is warned about as a construction defect.
     */
    struct DistanceSplit
    {
        double max_reachable = 0., avg_reachable = 0.;
        double max_pinned = 0.;
        size_t n_reachable = 0, n_pinned = 0;
        /// max_reachable, split by where it was measured. The two answer different questions:
        /// the vertex max says the boundary is in the wrong PLACE, the edge max says it is too
        /// COARSE to be in the right place -- and they call for different remedies (smoothing vs
        /// refinement), so a run that is not converging needs to know which it is. On the dragon
        /// the whole residual is in the second: 0.73% of delta at the vertices, 14% between them.
        double max_at_vertex = 0., max_in_edge = 0.;
        /// Reachable band vertices that have left the potential's support entirely, and the
        /// worst of them. Collected here rather than in a second traversal because
        /// check_offset_within_support() asks the same question of the same vertices.
        size_t n_outside_support = 0;
        size_t worst_outside_vid = static_cast<size_t>(-1);
        double worst_outside_dist = 0.;
    };
    DistanceSplit distance_deviation_split() const;

    /// The same split over the quantity the LOOP converges on: the Phi residual, as a length.
    /// Reported beside the Euclidean one so the two offsets can always be compared.
    DistanceSplit residual_split() const;
    /// Which vertices lie on the band's OUTER surface -- the one that is supposed to sit at
    /// target_distance. Shared by every measurement so they all agree on what "the band" is.
    std::vector<bool> band_vertex_mask() const;

    /// The furthest any offset-boundary vertex sits from the input complex, by BVH. 0 when no
    /// offset exists yet. Sizes dhat in init_offset_potential(); the 3D twin has the same name.
    double max_band_vertex_distance() const;
    /// |dist(vid, input complex) - target_distance|. DIAGNOSTIC: the Euclidean offset, which
    /// the level set only coincides with away from reentrant features.
    double band_vertex_distance_error(const size_t vid) const;

    /// How far vid is from the level set Phi = c, as a length. This is what the loop converges
    /// on and what the sizing field refines by.
    double band_vertex_residual(const size_t vid) const;

    /// The residual sampled at points ALONG a band edge -- see offset_edge_samples().
    struct EdgeSamples
    {
        double max = 0.;
        double sum = 0.;
        size_t n = 0;
    };

    /**
     * @brief The Phi residual at `offset_residual_samples` interior points of band edge `e`.
     *
     * THE CRITERION CANNOT BE A VERTEX CRITERION. Distance error at the vertices alone does not
     * pin down the offset: a boundary can have every vertex exactly on the level set while
     * zig-zagging or cutting corners between them, which reads as converged and is not the
     * offset. That is the same gap the paper's normal-deviation criterion (Sec. 5.3.3) covered,
     * and it is not hypothetical -- measured on topological_offset_2d_vertex_input, a band that
     * had decimated to twelve segments reported a vertex error of 0.2% of target_distance while
     * its edge midpoints sat at 18% of target_distance from the input; and on the dragon, 5.5%
     * at the vertices against 26% at the midpoints.
     *
     * Sampling the EDGES is what the potential makes possible and the old distance field did
     * not: Phi is defined everywhere, so the offset can be measured anywhere along the band
     * rather than only where the mesh happens to have put a vertex. The same samples feed
     * face_criterion_rel(), so the sizing field refines a band that is too coarse to represent
     * the offset instead of letting it decimate.
     *
     * Samples are uniform interior points, i/(k+1) for i = 1..k, so k = 1 is the midpoint.
     * Returns nothing for an edge with an UNREACHABLE endpoint: a segment running onto the
     * input complex is legitimately closer than target_distance along its length, and no
     * operation can change that.
     */
    EdgeSamples offset_edge_samples(const Tuple& e) const;

    /**
     * @brief Visit the same interior sample points offset_edge_samples() measures on.
     *
     * Factored out so the residual and the CONVERGENCE GRADIENT are measured on one lattice --
     * a criterion sampled on a different set of points from the quantity the sizing field
     * refines by is two measurements pretending to be one. The 3D twin is
     * for_each_offset_face_sample().
     */
    template <typename Visit>
    void for_each_offset_edge_sample(const Tuple& e, Visit&& visit) const
    {
        const int k = m_offset_params.offset_residual_samples;
        if (k <= 0) return;
        const Vector2d p0 = m_vertex_attribute[e.vid(*this)].m_posf;
        const Vector2d p1 = m_vertex_attribute[e.switch_vertex(*this).vid(*this)].m_posf;
        for (int i = 1; i <= k; ++i) {
            const double t = double(i) / double(k + 1);
            visit(Vector2d((1. - t) * p0 + t * p1));
        }
    }

    /**
     * @brief The convergence criterion's own split: ||grad (Phi - c)^2|| at band vertices --
     * the deciding measure -- plus the edge-interior chord diagnostic and the normal-aligned
     * reference quantity.
     *
     * PINNED VERTICES ARE REPORTED, NOT GATED, which is where this deliberately parts company
     * with residual_split(). A residual is a statement about the BOUNDARY: a pinned vertex
     * sitting off the level set is a real error in the offset the run returns. A gradient is a
     * statement about the ITERATION: it asks whether the vertices the optimizer can move have
     * stopped moving, and folding in one it never moves would make convergence unreachable by
     * construction.
     *
     * THE DECIDING MEASURE IS THE FULL GRADIENT NORM AT VERTICES: max_reachable
     * (== max_at_vertex) is max ||2 (Phi - c) grad Phi|| over reachable band vertices, the
     * exact quantity every Phase B local solve stops on, so the run's verdict and the visits'
     * stops are one test. max_in_edge is the EDGE-INTERIOR half of the criterion and GATES
     * alongside the vertex half (2026-08-23) -- see gradient_split() for why the earlier
     * "diagnostic only" reading was wrong. max_normal_aligned is |grad E . n| at vertices over
     * reachable AND pinned -- the quantity measure_gradient_reference() takes its max of.
     */
    struct GradientSplit
    {
        double max_reachable = 0., avg_reachable = 0.;
        double max_pinned = 0.;
        size_t n_reachable = 0, n_pinned = 0;
        /// max_at_vertex is the vertex half of the criterion; max_in_edge is the EDGE-INTERIOR
        /// half, and since 2026-08-23 it GATES TOO -- same quantity, same bar. Both are the full
        /// norm ||2 (Phi - c) grad Phi||, and max_in_edge counts only edges whose BOTH endpoints
        /// are reachable, so a chord to a pinned vertex cannot make the run unconvergeable.
        double max_at_vertex = 0., max_in_edge = 0.;
        /// The same edge measure over edges with an unreachable endpoint. Reported, never gating
        /// -- the pinned twin of max_pinned.
        double max_in_edge_pinned = 0.;
        /// max |2 (Phi - c) grad Phi . n| at band vertices, reachable AND pinned: the quantity
        /// the reference is the max of. See measure_gradient_reference().
        double max_normal_aligned = 0.;
        /// Edge-interior samples measured into max_in_edge (not part of n_reachable).
        size_t n_edge_samples = 0;
        /// Band vertices the smoother would refuse to place, so their gradient is not part of
        /// the fixed point this measures.
        size_t n_skipped_inverted = 0, n_skipped_unrounded = 0;
        /// The vertex carrying max_reachable, for the log. size_t(-1) until a reachable
        /// vertex is measured.
        size_t worst_vid = static_cast<size_t>(-1);
    };
    /// @param include_edge_samples false skips the edge-interior half (the expensive one). Only
    /// Phase B's own per-pass stop passes false, because placement cannot reduce a chord term;
    /// every convergence decision passes true. See phase_b_band_gradient_linf().
    GradientSplit gradient_split(bool include_edge_samples = true) const;

    /**
     * @brief Unit normal of the OFFSET SURFACE at band vertex `vid`, Voronoi-length weighted.
     *
     * The 2D analogue of an area-weighted vertex normal: the offset boundary is a polyline, so
     * each incident offset edge contributes its own unit normal weighted by HALF ITS LENGTH --
     * the part of the edge whose nearest boundary vertex is this one, which is exactly the
     * vertex's Voronoi cell along the curve. Normalised at the end, so only the weights' ratio
     * matters.
     *
     * This is the DISCRETE SURFACE's own normal, and it is what the REFERENCE is measured
     * along: measure_gradient_reference() takes max |2 (Phi - c) grad Phi . n| over the
     * initial offset vertices with this n, and the convergence bar is a fraction of that. (The
     * running tests -- the criterion and the Phase B local stops -- are the FULL gradient
     * norm; the in-edge chord diagnostic projects onto the edge's own unit normal.) It
     * replaced a projection back to the input complex, which measured against the INPUT's
     * geometry rather than the surface's own -- and flips discontinuously across the medial
     * axis where two offsets merge, exactly where this model family is interesting.
     *
     * Each edge normal is oriented OUTWARD -- away from the incident offset-band face -- so the
     * two contributions at a vertex reinforce instead of cancelling. Returns the zero vector for
     * a vertex with no live offset edge, or where the incident edges are degenerate.
     */
    Vector2d offset_surface_normal(const size_t vid) const;

    /**
     * @brief THE normal for an offset vertex. Every caller that needs one goes through here.
     *
     * There is exactly one definition of "the normal at an offset vertex" in the component, and
     * this is it -- so switching the definition is a one-line edit here rather than a hunt
     * through the call sites. Two implementations live in the body, selected at runtime by
     * WMTK_OFFSET_NORMAL so one build can measure both.
     *
     *  - DEFAULT (unset): project to the INPUT COMPLEX. n is the unit vector from the nearest point
     * on the complex to the vertex, i.e. the direction the offset grew along. It is a property of
     * the input geometry alone, so it does not move as the offset mesh is re-triangulated, and it
     * is defined for every band vertex whether or not it has live offset edges. Known weakness, and
     * the reason it was removed once before: it flips discontinuously across the MEDIAL AXIS, i.e.
     * exactly where two offset fronts approach each other -- which is where this model family is
     * interesting.
     *  - WMTK_OFFSET_NORMAL=surface: offset_surface_normal(), the Voronoi-length-weighted mean of
     * the incident offset edges' outward normals. Continuous across the medial axis, but a property
     * of the current triangulation rather than of the geometry, and it returns zero for a vertex
     * with no live offset edge.
     *
     * @return unit vector, or the zero vector where neither definition can produce one.
     */
    Vector2d offset_vertex_normal(const size_t vid) const;

    /// Measure m_gradient_reference -- max |2 (Phi - c) grad Phi . n| over the INITIAL
    /// offset-surface vertices, reachable and pinned alike -- and return the split it was
    /// measured from so the caller can report it without a second pass. Called once, before
    /// the A/B loop; see offset_gradient_tolerance().
    GradientSplit measure_gradient_reference();

    /// Turn a residual_split()'s outside-support tally into the hard error. Separate from
    /// check_offset_within_support() so the per-round check can reuse a split it already has.
    void report_outside_support(const char* when, const DistanceSplit& s) const;
    /// Whether vid is a band vertex the optimizer could still place at target_distance.
    ///
    /// Only the domain boundary disqualifies one. m_is_on_input USED TO as well, and that was
    /// wrong once freezing went: the flag is over-broad -- splits propagate it and collapses OR
    /// it onto survivors -- so a vertex carrying it may sit a full target_distance from the
    /// complex, which is to say exactly where the offset wants it.
    /// check_no_vertex_on_both_surfaces() already throws on the genuinely contradictory case,
    /// so any vertex reaching here with the flag is placeable, and booking it as pinned only
    /// hid it from the metric that decides convergence. Same change as 3D.
    bool band_vertex_is_reachable(const size_t vid) const
    {
        // INTERFERENCE-PINNED vertices are the second disqualification, and unlike the domain
        // wall it is MEASURED rather than structural: the vertex's own Gauss-Seidel step found
        // no descent, and Phi around it says the level set it is chasing does not exist there
        // (see pin_interference_stalled_vertices). Opt-in through WMTK_OFFSET_INTERFERENCE_PIN;
        // the flag is only ever set when that is on, so this test costs nothing when it is off.
        if (vid < m_interference_pinned.size() && m_interference_pinned[vid]) return false;

        // ENVELOPE-HELD OFFSET VERTICES ARE PINNED, and this is the default as of 2026-08-23.
        // A vertex on the offset front that a region envelope also holds must stay within
        // envelope_size of the input region boundary AND sit on Phi = c, which is
        // target_distance away. Those are not simultaneously satisfiable, so its residual
        // gradient is not a statement about the offset's quality -- it is a statement about the
        // constraint -- and leaving it in max_reachable makes convergence impossible by
        // construction. Measured on topo_annots_groups (tag_0 & tag_2, delta 1.2,
        // convergence_gradient_norm_rel 1e-3): FOUR such vertices held max_grad at 3.6 against a
        // bar of 0.0339 for all ten rounds while 115k operations ran around them, one of them at
        // Phi/c = 5.6. At the default rel of 0.2 the same run converges in one round, because the
        // bar sits above the conflict and it is invisible -- which is exactly why this needs to
        // be structural rather than left to whether the tolerance happens to hide it.
        //
        // STILL REPORTED, never silent: the pinned half of every band measure is logged beside
        // the reachable half, so these vertices stay visible as a count and a max. That is the
        // same contract vertex_is_on_domain_boundary() gets below.
        //
        // THIS IS A RETREAT, NOT A SOLUTION, and it should be reverted the moment a placement
        // that works exists. Three have been written and none does -- refusal freezes them,
        // projection-to-the-curve is strictly stronger than the constraint and made a run worse,
        // and the tangential arclength solve has an anchoring bug. smooth_before()'s Phase B
        // Offset branch carries the commented-out call to restore. See "OPEN PROBLEMS" in
        // .claude/CLAUDE.md.
        if (m_vertex_extra[vid].m_is_on_offset && vertex_boundary_mask(vid) != 0) return false;

        return !vertex_is_on_domain_boundary(vid);
    }

    /**
     * @brief Book offset vertices that stalled against ANOTHER offset front as unreachable.
     *
     * Called after every Phase B pass, and only when WMTK_OFFSET_INTERFERENCE_PIN is set.
     *
     * THE PROBLEM. Where two offset fronts approach the same curve, Phi in the corridor between
     * them never falls to c -- on two_circles at delta 0.1 the gap midpoint sits at Phi = 2c --
     * so there is no level set for those vertices to reach. They are not under-resolved and they
     * are not badly placed; their target does not exist. Left in max_reachable they make
     * convergence impossible by construction, and every round answers the residual by refining,
     * which cannot help: Phi is a function of the INPUT COMPLEX, not of the mesh, so no amount
     * of refinement moves the level set.
     *
     * THE TEST IS PURELY GEOMETRIC, and all three must hold. There WAS a fourth, first: that the
     * vertex's own placement had reported PlacementStop::MidStationary. It never fired once --
     * the damped Newton step's Armijo test (c1 = 1e-4) always admits some tiny length near a
     * ridge, so the vertex crawls instead of reporting no-descent, and the whole rule was inert
     * (measured on two_circles at delta 0.1: 231 of 231 visits `moved`, every pass, every round).
     * It was redundant anyway -- conditions 1-3 below already say the level set is not there,
     * whatever the solver reports -- and tying the rule to a stop reason tied it to a solver
     * detail. If a measured stall is ever wanted as corroboration, use the ACCEPTED DISPLACEMENT
     * against the local edge length, which no change of solver can invalidate.
     *
     *   1. Phi(x) > c. It is strictly inside the offset region, so it cannot be on the level set;
     *      a vertex outside is a different situation and is left alone.
     *   2. |grad Phi . n| <= kInterferenceTangentialRel * |grad Phi|. The field's gradient is
     *      essentially TANGENTIAL to the offset direction, i.e. no progress is available along n.
     *   3. n^T grad^2 Phi n > 0. Phi has a genuine local MINIMUM along n -- which is what makes
     *      this an interference ridge rather than the flat far field (where grad Phi vanishes
     *      identically and the second derivative with it) or an inflection.
     *
     * Condition 3 is the one that earns the design. Without it the test also matches the region
     * beyond dhat, where Phi == 0 and grad Phi == 0 -- exactly the spurious minimum that the
     * {Phi >= c} constraint was added to exclude from the placement.
     *
     * IT NEVER THROWS, by instruction. A large pinned count is reported loudly -- count, share of
     * the surface, and the worst Phi/c excess -- because this lets a run converge by declaring
     * part of the surface impossible. That is correct where the offset genuinely self-intersects
     * and would mask a real failure where it does not, so the number has to be read, not trusted.
     *
     * @return how many vertices are pinned after this pass (recomputed from scratch each time --
     *         a vertex that stalls in one pass may move in the next).
     */
    size_t pin_interference_stalled_vertices(size_t pass);

    /// See pin_interference_stalled_vertices(). Indexed by vid; empty when the feature is off.
    /// Cleared at the top of each round's Phase A so a stale flag never rides a recycled slot
    /// through a topology change, and recomputed from scratch after every Phase B pass.
    std::vector<char> m_interference_pinned;

    /// Which offset vertices reported PlacementStop::MidStationary in the pass just finished.
    /// Written once per vertex per pass by smooth_offset_vertex_backtracking(), so parallel
    /// sweeps do not race on an index.
    std::vector<char> m_placement_stalled;
    /// Set by the placement when a vertex's last visit stopped on QualityBound, cleared when it
    /// moved. distance_criterion() counts such a vertex as placed -- its level set is
    /// unreachable by construction -- and drops edges touching one from the resolution and
    /// orientation halves; update_band_sizing_from_tolerance() does not refine such edges.
    std::vector<char> m_placement_pressed;
    std::atomic<int> m_phase_b_pressed{0}; ///< per pass: placements that stopped on QualityBound

    /// |grad Phi . n| / |grad Phi| below which the field's gradient counts as TANGENTIAL to the
    /// offset direction. Hard-coded rather than a spec key: it is a near-degeneracy test, not a
    /// tuning knob, and the measured value is reported per vertex so a wrong choice is visible.
    static constexpr double kInterferenceTangentialRel = 0.1;

    /**
     * @brief TriWild's stall-driven sizing refinement, verbatim.
     *
     * Structurally TriWildMesh::refine_sizing_around_worst, down to the shared helpers in
     * wmtk/utils/SizingField.hpp and every stuck_refine_* parameter: rank faces by AMIPS,
     * force-split the worst ones' longest edges, grow the region by rings, lower the
     * per-vertex sizing scalar, grade it outward.
     *
     * PHASE A ONLY: mesh_improvement() is its one caller, and the alternating driver only runs
     * that as Phase A. The Phase B branch that ranked by face_criterion_rel() was unreachable
     * there and is deleted (2026-08-22); Phase B's refinement question belongs to
     * update_band_sizing_from_tolerance().
     */
    size_t refine_sizing_around_worst(double max_metric) override;

    /**
     * @brief Why Phase A is stuck: a census of the faces stuck-refine is about to chase.
     *
     * Runs from refine_sizing_around_worst(), which only fires once max energy has stalled. The
     * question it answers is not "how bad is the mesh" -- the log already says 1e50 -- but
     * whether refinement is even the right response, and it separates four things the single
     * MAX_ENERGY sentinel fuses:
     *
     *  - EXACTLY inverted (is_inverted) vs merely float-degenerate (is_inverted_f only). The
     *    second kind is a valid triangle AMIPS2D cannot score because m_posf lost the area; it
     *    is a rounding problem, not a geometry one, and splitting it makes two of them.
     *  - carrying an UNROUNDED vertex, where m_posf is the wrong number outright.
     *  - already BELOW the split gate, i.e. the longest edge is shorter than l * sbar * 4/3, so
     *    the next pass cannot split them at all and lowering the sizing field is pure waste.
     *  - at the sizing FLOOR, where apply_sizing_refinement has nothing left to give.
     *
     * Plus where they are: class distribution, connected clusters, and how much the set overlaps
     * the previous call's (quantised on an l/10 grid, because fids are recycled and cannot be
     * compared across passes). A high overlap with a low cluster count says the pass is chasing
     * the same few spots forever; a scattered, changing set says something is manufacturing new
     * degeneracies as fast as they are refined.
     */
    void log_stuck_refine_census(double max_metric, double filter_energy);

    /**
     * @brief For every element above `filter_energy`, WHY its edges cannot be split.
     *
     * log_stuck_refine_census() answers "what are the bad elements"; this answers "what is
     * stopping the mesh from fixing them", which is a different question and the one that
     * matters when phase A refines somewhere else instead.
     *
     * Attributes each of a bad face's three edges to the FIRST gate that refuses it, in the
     * order the code applies them (TriOptimizerMeshSplit.cpp):
     *
     *   short     length^2 < splitting_l2 * mean(sizing)^2 -- never even offered to the queue.
     *             The remedy is the SIZING FIELD, not the split.
     *   valence   a link vertex is over split_high_valence_threshold. Reported as a ceiling:
     *             the real gate is one such split per vertex per pass, which a static probe
     *             cannot see, so this counts vertices that COULD be refused, not that were.
     *   contain   the dispatched envelope refuses one of the two halves. This is the
     *             "envelope too tight" hypothesis, and the column says WHICH envelope.
     *   free      nothing blocks it -- so a face all of whose edges are `free` is not being
     *             starved by any gate, and the stall is elsewhere (priority, valence budget,
     *             or a shape no split can fix).
     *
     * Two shortcuts, both stated so the output is not over-read:
     *
     *  - THE MIDPOINT CANNOT INVERT A HEALTHY PARENT. Splitting (a,b) in face (a,b,c) makes
     *    (a,m,c) and (m,b,c), each with exactly half the parent's signed area, so the base's
     *    exact inversion check at TriOptimizerMeshSplit.cpp:250 can only fire on a parent that
     *    is ALREADY inverted. The census reports the parent's own inversion instead of probing.
     *  - THE ENVELOPE OF A CHILD SEGMENT IS THE PARENT'S. surface_envelope_for_edge dispatches
     *    on edge_mask(), the AND of the endpoints' masks, and the midpoint's mask is itself the
     *    AND of the parent's endpoints -- so mask(a,m) == mask(m,b) == mask(a,b) and one
     *    dispatch serves both halves. This is the same reasoning split_adjust_position relies on.
     *
     * Each bad face is also located: centroid, distance to the input complex, and Phi/c there.
     * That is what separates "the collided corridor between two fronts" from "somewhere random
     * in the ambient background", which is the distinction the whole census exists to draw.
     *
     * Diagnostic only: reads the mesh, writes only the log.
     */
    void log_refine_block_census(const std::string& when, double filter_energy) const;

    /**
     * @brief Instrumentation only: which operation manufactures the MAX_ENERGY needles.
     *
     * The base leaves two doors open and this counts what goes through each.
     *
     *  - A SPLIT is never refused on quality (TriOptimizerMeshSplit.cpp: "A split is never
     *    refused on quality" -- it checks orientation, rounding and containment only). Every
     *    face incident to a freshly split midpoint is a face that split just created, so
     *    needles counted there in split_after_vertex() were created by it.
     *  - A COLLAPSE is admitted by collapse_quality_allowed() when `q <= ring_max`, and once a
     *    single needle sits in the ring, ring_max IS MAX_ENERGY -- so the clause admits a
     *    collapse that produces another needle. That is a contagion, and the three counters
     *    below say which clause did the admitting.
     */
    bool collapse_quality_allowed(size_t v1, size_t v2, double q, double ring_max) const override;

    mutable std::atomic<size_t> m_deg_split_created{0};
    mutable std::atomic<size_t> m_deg_collapse_offered{0};
    mutable std::atomic<size_t> m_deg_collapse_allowed{0};
    mutable std::atomic<size_t> m_deg_collapse_by_ringmax{0};
    mutable std::atomic<size_t> m_deg_collapse_by_stop{0};
    mutable std::atomic<size_t> m_deg_collapse_by_unrounded{0};
    /// Values at the previous census, so each census can report DELTAS rather than totals.
    std::array<size_t, 6> m_deg_prev_counts{{0, 0, 0, 0, 0, 0}};

    /**
     * @brief Where the FIRST needles come from -- a tripwire, not a census.
     *
     * The census counts the population once it exists and the attribution counters say which
     * operation touches them; neither says how the first one is born, and force-splitting only
     * explains the doubling AFTER there is something to double. This logs the first
     * kNeedleReports needle faces any operation hook sees, with everything needed to tell a
     * creation from a copy: the operation, the parent quality where there is one, both the
     * float and the exact orientation, full-precision coordinates, and each vertex's surface
     * flags, birth epoch and rounding state.
     *
     * Deliberately capped -- once the force-split loop engages there are thousands per pass and
     * the log stops being readable. It is the first few that carry the information.
     */
    void report_needle(const char* op, size_t fid, double parent_q) const;
    static constexpr size_t kNeedleReports = 12;
    /**
     * @brief What counts as a needle for the tripwire -- deliberately far below MAX_ENERGY.
     *
     * The first pass of this instrumentation tested `>= MAX_ENERGY` and found the first such
     * face already had a PARENT at AMIPS 3.45e14: catastrophically flat, entirely invisible to
     * a 1e50 test, and the actual origin of the collinearity. A healthy triangle is O(2); 1e6
     * is far outside anything the optimizer should tolerate and far below the sentinel, so the
     * creation event is caught while its parent is still scoreable and can be quoted.
     */
    static constexpr double kNeedleQuality = 1e6;
    mutable std::atomic<size_t> m_needle_reports{0};

    /// Population scan at a named moment, for the points no operation hook covers -- after the
    /// pre-pass, after construction, at each collapse pass. Reports the count and the worst few.
    void needle_scan(const char* when) const;
    /// Diagnostic only: the base offers no per-iteration hook except this one, so the needle
    /// population scan rides on it. Calls nothing else -- the base default is empty.
    void collapse_pass_begin() override;

    /// Quantised centroids of the MAX_ENERGY faces at the previous stuck-refine, for the overlap
    /// line above. Diagnostic only; nothing reads it but log_stuck_refine_census().
    std::set<std::pair<long, long>> m_stuck_prev_cells;
    size_t m_stuck_calls = 0;

    // optimization_bare_coarsen_passes() is NOT overridden: the base's true -- TriWild's opening
    // and closing unlimited-length collapse passes -- applies, as in 3D. The override returned
    // false on the grounds that the offset boundary "has no envelope holding it, so a bare
    // collapse pass can only decimate it". Phase A now rebuilds m_offset_envelope before every
    // mesh_improvement() call, so the boundary is held exactly the way TriWild's input surface
    // is during ITS bare passes, and the premise is gone. Decimating to what the envelope
    // tolerates is the contract, not the leak.

    /**
     * @brief A collapse is accepted by the SAME criterion the smoothing minimises.
     *
     * The smoother places an offset vertex by minimising w (Phi - c)^2 and the loop converges
     * when the Phi residual is inside tolerance along the whole offset boundary, vertices and
     * edge interiors alike. Every other operation has to answer to that same measure, or it can
     * undo in one collapse what the smoother spent an iteration achieving. Length gates cannot
     * express it: they ask whether an edge is short relative to a sizing target, which is a
     * statement about the MESH, while the criterion asks whether the boundary is still the
     * offset, which is a statement about the GEOMETRY -- and only the second is what the run is
     * for.
     */
    bool collapse_edge_after(const Tuple& t) override;
    /// Max of the two normalized criteria (AMIPS over stop, residual over tolerance) on this
    /// face; >= 1 means it fails at least one. The coarsen-mode collapse accept reads it, and
    /// it is the per-face form of optimization_quality_stats()'s Phase B max.
    double face_criterion_rel(const size_t fid) const;
    /**
     * @brief Put the frames beside the run's own output, and RENAME THEM INTO ONE TIMELINE.
     *
     * Every debug frame in 2D goes through here -- the per-pass ones the shared
     * local_operations() driver writes during phase A, the per-pass ones phase_b_smooth()
     * writes, and the two per-phase ones the A/B driver writes -- so this is the one place that
     * can give them all a common order. The driver hands a bare `debug_N` or `phase_<r><A|B>`;
     * both come out as
     *
     *     <output>_step_<NNNNN>_r<round><A|B><pass>.vtu      a pass inside a phase
     *     <output>_step_<NNNNN>_r<round><A|B>_end.vtu         the frame the phase handed on
     *
     * NNNNN is a single monotonic counter across the whole run, so sorting on it is run order,
     * full stop -- no interleaving two series by timestamp, no guessing whether a `debug_` frame
     * came before or after a `phase_` one.
     *
     * <pass> counts passes WITHIN the current phase and restarts whenever the round or the
     * phase changes, so `r1A3` reads directly as "round 1, phase A, third pass" -- which is what
     * a viewer puts on a slider position. The restart is detected here rather than signalled by
     * the driver, so no call site has to remember to reset anything.
     *
     * Round 0 is construction: the pre-loop frame is written before m_ab_round is set.
     *
     * Renaming HERE rather than at the call sites is what keeps this out of the shared driver:
     * wmtk::TriOptimizerMesh writes phase A's per-pass frames and is also triwild's and
     * simwild's, so its naming is not ours to change.
     */
    void write_smoothing_debug_output(const std::string& path) const override
    {
        std::string name = path;
        const char ph = (m_phase == OptPhase::A) ? 'A' : 'B';
        if (m_ab_round != m_debug_last_round || ph != m_debug_last_phase) {
            m_debug_last_round = m_ab_round;
            m_debug_last_phase = ph;
            m_debug_pass = 0;
        }
        if (path.rfind("debug_", 0) == 0) {
            // The pass this frame follows, from TriOptimizerMesh::m_debug_pass_name: the shared
            // driver writes several frames per operation group (a checkpoint after every group,
            // even one that ran nothing, plus the group's own frame), and Phase B writes one per
            // sub-sweep, so without the name a timeline reads as identical frames in a row.
            name = fmt::format(
                "step_{:05d}_r{}{}{}{}",
                m_debug_seq++,
                m_ab_round,
                ph,
                ++m_debug_pass,
                m_debug_pass_name.empty() ? std::string() : "_" + m_debug_pass_name);
        } else if (path.rfind("phase_", 0) == 0) {
            name = fmt::format("step_{:05d}_r{}{}_end", m_debug_seq++, m_ab_round, ph);
        }
        const_cast<TopoOffsetTriMesh*>(this)->write_vtu(m_offset_params.output_path + "_" + name);
    }

    /**
     * @brief initialize TriMesh from vertex, face, tag data
     * @param V: #V by 2 vertex matrix
     * @param F: #F by 3 face matrix
     * @param F_tags: #F by #physical groups tag matrix
     * @param V_env: #V_env by 2 EnvelopeSurface vertex matrix
     * @param F_env: #F_env by 2 EnvelopeSurface edge matrix
     */
    void init_from_image(
        const MatrixXd& V,
        const MatrixXi& F,
        const MatrixSi& F_tags,
        const MatrixXd& V_env,
        const MatrixXi& F_env,
        const std::vector<std::string>& tag_names);

    /**
     * @brief ensure ambient tag does not overlap any other tags in mesh.
     */
    bool ambient_assert();

    /**
     * @brief label input complex simplices as per boolean expression (or single body mode)
     */
    void label_input_complex();

    /**
     * @brief check if the input complex is empty. Only valid after calling init_from_image(...).
     * Checks if any vertices (therefore any simplices) are labelled 1, if not returns true
     */
    bool empty_input_complex();

    /**
     * @brief Build the input complex's BVH and its smooth offset potential, from one extraction.
     *
     * Must be called after init_from_image(...) and label_input_complex(). The potential is
     * built from the BOUNDARY of the complex rather than its interior: Phi's 2D primitives are
     * segments and points, so a solid input region enters as its outline. Outside the region --
     * which is the only place an offset exists -- the two descriptions agree exactly.
     */
    void init_input_complex_bvh();

    /**
     * @brief Build the smooth offset potential from the extraction init_input_complex_bvh() kept.
     *
     * Separate from that call only because it needs target_distance and offset_dhat_factor, which
     * a caller wanting nothing but the distance field has no reason to have set. The GEOMETRY is
     * still extracted exactly once, so the potential and the BVH cannot describe different inputs.
     */
    void init_offset_potential();

    /// The complex as the potential sees it: vertices, its BOUNDARY segments, and its isolated
    /// points. Filled by init_input_complex_bvh(), consumed by init_offset_potential().
    MatrixXd m_phi_V;
    MatrixXi m_phi_E;
    std::vector<int> m_phi_P;

    //// overriden splits/invariants
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
    bool invariants(const std::vector<Tuple>& tris) override;
    //// overriden splits/invariants

    /**
     * @brief entry point for offset procedure
     */
    /**
     * @brief TriWild over the INPUT mesh, before any of the offset exists.
     *
     * Runs the shared mesh_improvement() with Phase A's own parameters and units, at a point in
     * the pipeline where the only tracked surfaces are the tag-region boundaries (the input
     * complex and the domain wall among them) and the only containment is their per-tag
     * envelopes -- built in init_surfaces_and_boundaries(), which has already run. There is no
     * offset yet, so there is no offset envelope and no Phi term: this is TriWild, exactly.
     *
     * WHY IT IS WORTH A PASS. marching_tris() puts the offset boundary on the background
     * triangulation's OWN cell boundaries, so the distance the constructed offset lands from the
     * input complex is a property of the input mesh rather than of target_distance -- and
     * init_offset_potential() then sizes dhat from that reach. One coarse cell touching the
     * complex is enough to inflate dhat several-fold, and a large dhat merges the level sets of
     * features that are close together, which no later stage can undo. Improving the mesh the
     * marching runs on is the earliest place to attack that.
     *
     * THE SIZING FIELD IS SEEDED FIRST, to target_distance on the INPUT-COMPLEX BOUNDARY and
     * graded outward from there. That is the point of the pass: a band built on cells of delta
     * scale lands near delta from the complex, which is what keeps dhat small. Seeding is also
     * not optional in the weaker sense -- the base field is 1 everywhere, a target of
     * m_params.l, which is far coarser than the mesh around the complex (two_circles: l = 0.329
     * against incident lengths of 0.078), so an unseeded pass would COARSEN the input and leave
     * the marching a worse mesh than it started with.
     */
    void pre_optimize_input_mesh();

    void execute_offset(const std::filesystem::path& output_file);

    /**
     * @brief execute simplistic marching tris. All edges with one vertex labelled 0 and the other
     * 1/2 are split, always at the MIDPOINT.
     *
     * The other two modes are gone. Splitting the marched edge at the root of d(l) -
     * target_distance is not what the paper does (Sec. 5.2 places inserted vertices at the
     * midpoint and leaves the distance to Step 3), and the ::Initial mode that stepped
     * target_distance/2 in from the background end went with it: no target_distance enters
     * construction at all now, and carrying the boundary out to the level set is entirely the
     * optimization phase's job. Same as 3D.
     */
    void marching_tris();


    //// simplicial embedding stuff
    /**
     * @brief check if the input complex (simplices labelled 1) are simplicially embedded w.r.t. the
     * entire mesh
     */
    bool is_simplicially_embedded() const;

    /**
     * @brief check if a triangle satisfies simpicial embedding criteria w.r.t. input complex
     * (simplices labelled 1)
     */
    bool tri_is_simp_emb(const Tuple& t) const;

    /**
     * @brief make mesh a simplicial embedding of the input complex (simplices labelled 1)
     */
    void simplicial_embedding();
    //// simplicial embedding stuff

    /**
     * @brief update 'tags' data for triangles in the offset region (tris labelled 2) based on
     * the given offset tag values in m_offset_params.offset_tag_value
     */
    void set_offset_tri_tags();

    /**
     * @brief verify that the closed offset region (simplices labelled 1 or 2) form a manifold
     * region. This should be true for any offset. This function is for verification
     */
    bool offset_is_manifold();

    //// output stuff
    /**
     * @brief Sample the smooth offset potential on a dense grid and write it as `<path>_phi.vtu`.
     *
     * The offset is a level set of a field defined everywhere, and the output mesh only ever
     * samples that field along one curve -- so a result that looks wrong is impossible to
     * diagnose from the mesh alone. This writes the field itself: Phi (clamped, since it
     * diverges on the input complex), the residual as a length, and the exact Euclidean distance
     * beside it, all as VERTEX fields on a triangulated grid so a viewer can draw the isoline
     * Phi = c directly and compare it against the Euclidean offset in the same window.
     *
     * @param n samples per side; 0 or 1 writes nothing.
     */
    void write_phi_grid(const std::string& path, int n) const;

    void write_input_complex(const std::string& path);
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
        Vector2d new_v_pos;
        VertexExtra2d new_v_extra;

        // cache edge attributes
        EdgeSnapshot2d split_eattr;
        std::map<simplex::Edge, EdgeSnapshot2d> existing_eattr;

        // cache face attributes
        std::map<size_t, FaceSnapshot2d> opp_v_fattr;
    };
    wmtk::threading::enumerable_thread_specific<EdgeSplitCache> edge_split_cache;

    struct FaceSplitCache
    {
        size_t v1_id;
        size_t v2_id;
        size_t v3_id;
        Vector2d new_v_pos;
        VertexExtra2d new_v_extra;

        std::map<simplex::Edge, EdgeSnapshot2d> existing_eattr; // 3 orig edges
        FaceSnapshot2d split_fattr; // split face attributes
    };
    wmtk::threading::enumerable_thread_specific<FaceSplitCache> face_split_cache;

private: // helpers
    /**
     * @brief determine if any tag from tag1 is also present in tag2.
     */
    bool any_tag_present(const CellTag& tag1, const CellTag& tag2) const
    {
        for (const int64_t& i : tag1) {
            if (tag2.find(i) != tag2.end()) {
                return true;
            }
        }
        return false;

        // if (tag2.empty()) {
        //     return tag1.empty();
        // }
        // if (tag1.empty()) { // tag1 is ambient and tag2 is not
        //     return false;
        // }

        // for (const int64_t& i : tag1) {
        //     if (tag2.find(i) != tag2.end()) {
        //         return true;
        //     }
        // }
        // return false;
    }

    /**
     * @brief sort vector of edge simplices in place by decreasing length
     */
    void sort_edges_by_length(std::vector<simplex::Edge>& edges)
    {
        std::sort(
            edges.begin(),
            edges.end(),
            [this](const simplex::Edge& e1, const simplex::Edge& e2) {
                double len1 = (m_vertex_attribute[e1.vertices()[0]].m_posf -
                               m_vertex_attribute[e1.vertices()[1]].m_posf)
                                  .squaredNorm();
                double len2 = (m_vertex_attribute[e2.vertices()[0]].m_posf -
                               m_vertex_attribute[e2.vertices()[1]].m_posf)
                                  .squaredNorm();
                return len1 > len2;
            });
    }

public: // helpers
    /**
     * @brief get global id of edge from simplex::Edge object
     */
    size_t edge_id_from_simplex(const simplex::Edge& e) const
    {
        const auto& verts = e.vertices();
        const auto incident = simplex_incident_triangles(e);
        const auto& faces = incident.faces();

        assert(!faces.empty()); // throw error here otherwise

        const size_t f_id = tuple_from_simplex(faces.front()).fid(*this);
        const Tuple t_edge = tuple_from_edge(verts[0], verts[1], f_id);
        return t_edge.eid(*this);
    }

    /**
     * @brief get Tuple simplex::Edge object
     */
    Tuple get_tuple_from_edge(const simplex::Edge& e) const
    {
        const auto& v = e.vertices();
        const auto faces = simplex_incident_triangles(e).faces();
        assert(!faces.empty());
        const size_t fid = tuple_from_simplex(faces.front()).fid(*this);
        return tuple_from_edge(v[0], v[1], fid);
    }

    /**
     * @brief get faces (as Tuples) that are edge-adjacent to the given face (as Tuple)
     */
    std::vector<Tuple> get_edge_adjacent_faces(const Tuple& f) const
    {
        std::vector<Tuple> adj_tris;
        auto tri_1 = f.switch_face(*this);
        if (tri_1) {
            adj_tris.push_back(tri_1.value());
        }
        auto tri_2 = f.switch_edge(*this).switch_face(*this);
        if (tri_2) {
            adj_tris.push_back(tri_2.value());
        }
        auto tri_3 = f.switch_vertex(*this).switch_edge(*this).switch_face(*this);
        if (tri_3) {
            adj_tris.push_back(tri_3.value());
        }
        return adj_tris;
    }
};


} // namespace wmtk::components::topological_offset
