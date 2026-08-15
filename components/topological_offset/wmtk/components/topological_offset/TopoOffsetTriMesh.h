#pragma once
#include <wmtk/TriMesh.h>
#include <wmtk/TriOptimizerMesh.h>
#include <wmtk/optimization/solver.hpp>
#include <algorithm>
#include <atomic>
#include <functional>
#include <set>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include "Parameters.h"
#include "SimplicialComplexBVH.hpp"

using CellTag = std::set<int64_t>;


namespace wmtk::components::topological_offset {


const int64_t TEMP_OFFSET_TRI_TAG = -1;
const CellTag TEMP_OFFSET_TRI_TAG_SET{TEMP_OFFSET_TRI_TAG};

/**
 * @brief AVERAGE distance error, as a fraction of target_distance, above which a non-converged
 * run with respect_all_topologies is reported as topologically blocked.
 *
 * Purely diagnostic -- it gates a log line and nothing else, which is why it is a constant here
 * rather than a JSON parameter. Deliberately loose: on the dragon rectangle every non-blocked
 * run sits at 0.06%-0.19% of target_distance and the blocked ones at 28%-36%, so anything in
 * [1%, 20%] gives the same answer. See the table at the call site in optimize_offset().
 */
constexpr double TOPOLOGY_BLOCK_AVG_FRAC = 0.05;

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
 * Three surfaces are tracked, each its own m_surface_class. The input complex is
 * INPUT_SURFACE_CLASS and is frozen outright. The offset boundary is OFFSET_SURFACE_CLASS: in 2D
 * it is exactly the set of edges across which the incident FACE LABELS differ -- there is no
 * stored definition of it, it falls out of the labelling, which is why label_offset_boundary()
 * recomputes it once at the top of the optimization. Every other tag boundary is
 * REGION_SURFACE_CLASS, tracked so the shared swap will not flip across it and silently move a
 * region, but otherwise an ordinary part of the background mesh. The latter two may move, within
 * m_envelope; only the offset one is driven toward target_distance.
 */
class TopoOffsetTriMesh : public wmtk::TriOptimizerMesh
{
public: // mode for splitting in marching tets
    enum class EdgeSplitMode {
        Midpoint = 0, // used for simplicial embedding steps
        Initial = 1, // this is used to initialize the complex. Its a little hacky
        Optimization = 2 // the optimization phase; the shared engine places the vertex
    };

public:
    int m_vtu_counter = 0;
    std::array<size_t, 3> m_init_counts = {{0, 0, 0}};
    size_t m_tags_count;
    /**
     * @brief The input complex as loaded. BUILT ONCE, NEVER REBUILT.
     *
     * Every offset quantity is measured against this: the distance error, the offset field's
     * normals, project_offset_vertex()'s target, the sizing score. init_input_complex_bvh() has
     * exactly one call site, in the driver, before execute_offset() runs -- so this holds the
     * original geometry however the mesh elements representing the complex are later refined,
     * coarsened or smoothed.
     *
     * That invariant is load-bearing and easy to break by accident. Since the complex stopped
     * being frozen, rebuilding this from the live mesh would redefine the offset distance in
     * terms of a surface the optimizer had just moved, and the convergence criterion would then
     * be measuring the mesh against itself.
     */
    SimplicialComplexBVH m_input_complex_bvh;
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
     * @brief SurfaceTagAttributes::m_surface_class: which of the three tracked surfaces an edge
     * belongs to.
     *
     * These have to be distinct classes, not one lumped "not the input complex". OFFSET is the
     * surface the optimization is trying to place at target_distance; REGION is every other tag
     * boundary in the file -- another body's outline, an overlap seam -- which is tracked for a
     * completely different reason (the shared swap copies one incident face's tags onto both
     * faces it creates, so an unguarded flip across a tag boundary silently moves that region),
     * and which has no business being pushed toward the offset distance. Filing both under
     * OFFSET is what sent project_offset_vertex() at vertices sitting ~40x target_distance from
     * the input complex, and what left update_sizing_field() refining around them forever.
     */
    static constexpr int INPUT_SURFACE_CLASS = 0;
    static constexpr int OFFSET_SURFACE_CLASS = 1;
    static constexpr int REGION_SURFACE_CLASS = 2;

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
    bool edge_is_region_boundary(const size_t eid) const
    {
        return m_edge_attribute[eid].m_is_surface_fs &&
               m_edge_attribute[eid].m_surface_class == REGION_SURFACE_CLASS;
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
     * are set at construction -- the complex by label_input_complex(), the band by conservative
     * growth -- from geometry, not from tags, and every operation now carries the label onto
     * the faces it creates, so this is exact.
     *
     * It used to read the TAGS, which cannot express the distinction the label exists for:
     * the band is named by the output tag, and nothing stops that tag already appearing
     * somewhere else in the input mesh. Every such face would then read as offset band, its
     * boundary would classify as OFFSET rather than REGION, and project_offset_vertex() would
     * drag an unrelated region toward target_distance.
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

    /// Move an offset-boundary vertex back onto the target distance from the input complex,
    /// blended with the Laplacian of its offset-boundary neighbours.
    bool project_offset_vertex(const Tuple& t);

    /**
     * @brief The direction the REGION curve runs through this vertex, if it has one.
     *
     * Only region-class edges count -- the offset boundary is exempt from the envelope, so its
     * direction is not a wall to slide along. Two incident region edges give the chord between
     * the two neighbours (the discrete tangent of a curve passing through); one gives that
     * edge's own direction (a curve terminating here, e.g. where a region outline meets the
     * offset). Zero or three or more is not a curve -- returns false, and the caller keeps
     * whatever the scaled search gave it.
     *
     * @param[out] tang unit tangent, written only on success. Sign is arbitrary; callers
     *                  project onto it, which is sign-independent.
     */
    bool region_boundary_tangent(const Tuple& t, Vector2d& tang) const;

    /**
     * @brief Put a vertex at the distance-error minimum along its region tangent.
     *
     * A vertex on a region boundary is effectively constrained to that curve -- the envelope
     * admits motion along it and almost none across it -- so the right thing to solve is the 1D
     * problem ON the curve: minimise |dist(p, input complex) - target_distance| over
     * p = p0 + s*tang. What this replaces is a single step of length (target - p0).tang, i.e.
     * the desired move PROJECTED onto the tangent, which fixes a direction and then takes an
     * arbitrary length in it.
     *
     * The measurement that motivated this, over the 122 slide events of the dragon rectangle at
     * target_distance_rel 5e-3: the projected step was never the best point on its own line.
     * 108 of 122 undershot and 14 overshot, and the best feasible point on the line had a median
     * of 52x lower error (worst case 36295x). The envelope was not what stopped them -- zero of
     * the 122 were blocked before the projected step, and 113 were still feasible at 20x it. The
     * vertices were under-stepped, not locked, which is also why splitting their incident region
     * edges would not have helped: a split adds samples to the curve, it does not enlarge the set
     * of positions the envelope admits.
     *
     * Bracket first, then minimise. `rejected` is the caller's feasibility predicate (envelope
     * plus inversion, evaluated at whatever position the vertex currently holds); the feasible
     * extent is found by geometric expansion out to `cap` and bisection of the first refusal,
     * separately in each direction, then golden-section search runs on the bracket with
     * infeasible probes scored as infinite. `cap` bounds how far along the curve a single pass
     * may travel.
     *
     * Leaves the vertex at the minimiser and returns its signed offset s along `tang`.
     */
    double minimize_distance_along_tangent(
        const size_t vid,
        const Vector2d& p0,
        const Vector2d& tang,
        const double cap,
        const std::function<bool()>& rejected);

    /// The 2D optimization phase: split / collapse / swap / smooth on the shared driver.
    void optimize_offset(const std::filesystem::path& output_file);

    /// Whether this face carries one of the offset output tags, i.e. is inside the offset band.
    /// Read from the tags, which every shared operation maintains, rather than from the face
    /// label, which is only refreshed once per optimization iteration.
    bool face_is_offset_band(const size_t fid) const;

    /**
     * @brief Live region-boundary test for an edge that currently exists in the mesh.
     *
     * Unlike edge_is_region_boundary(), which reads m_edge_attribute[].m_surface_class -- a
     * snapshot label_offset_boundary() only refreshes once per optimization ITERATION -- this
     * recomputes the same rule label_offset_boundary() applies, straight from the current face
     * tags, every time it is called.
     *
     * This is what surface_envelope_for_edge() and region_boundary_is_outside_envelope() must
     * use, not the cached class: split, collapse and swap all run between one relabelling and
     * the next, and collapse in particular re-points a vertex's neighbours onto brand-new edge
     * slots whose cached m_surface_class was never set (it defaults to 0, i.e. INPUT/untracked).
     * Querying the cache on such a segment silently reports "not region", the containment check
     * short-circuits to "not outside" without ever comparing a position, and a genuinely
     * bulging collapse is accepted. That is why sanity-check violations on this mesh appeared
     * immediately after the first collapse pass and nowhere before it.
     */
    bool edge_is_region_boundary_live(const Tuple& t) const
    {
        const std::optional<Tuple> opp = t.switch_face(*this);
        if (!opp) return false; // domain boundary: frozen, never a region boundary
        const size_t fa = t.fid(*this), fb = opp->fid(*this);
        if (face_is_input_complex(fa) != face_is_input_complex(fb)) {
            return false; // input-complex boundary: frozen, not the envelope's job
        }
        if (face_is_offset_band(fa) != face_is_offset_band(fb)) {
            return false; // offset boundary: exempt by design, it is what the optimization moves
        }
        return m_face_attribute[fa].tags != m_face_attribute[fb].tags;
    }

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

    /**
     * @brief How far the offset boundary faces from where it should be: {max, avg} in DEGREES.
     *
     * The angle between an offset-boundary edge's own normal and the direction the distance
     * field says it should be facing -- the unit vector from the nearest point on the input
     * complex out to a sample on the edge. Measured over the same band-outer-surface edges
     * compute_distance_deviation() measures vertices over, and reported alongside it.
     *
     * Distance error alone does not pin down the offset: a boundary can have every vertex at
     * exactly target_distance while zig-zagging between them, which reads as converged but is
     * not the offset. The normal deviation is what sees that, which is why the paper's own
     * Termination criterion (Sec. 5.3.3) tests sigma_max alongside the distance errors.
     */
    std::pair<double, double> compute_normal_deviation() const;

    /**
     * @brief Paper Definition 5 for a 1-simplex: max angle between the offset normal at the
     * edge's center and the offset normal at p_i = 0.1*p_c + 0.9*p_v, in degrees.
     *
     * Note this is NOT what the 3D branch's face_normal_deviation() computes -- that compares
     * the element's own geometric normal against the field. Both terms here are field normals,
     * which is what makes the quantity vanish under refinement and therefore something the
     * sizing field can actually drive to zero.
     */
    double edge_normal_deviation(const Tuple& e) const;

    /// The offset normal at an arbitrary point: the unit vector from the nearest point on the
    /// input complex out to `p`. False when `p` sits on the complex, where it is undefined.
    bool offset_field_normal(const Vector2d& p, Vector2d& n) const;

    /// Whether this edge is on the band's OUTER surface, recomputed from the tags on every call
    /// -- the live counterpart of edge_is_offset(), for use inside the operation passes.
    bool edge_is_offset_surface_live(const Tuple& e) const;

    /// Worst normal deviation over the offset-surface edges incident to `vid`, in degrees. The
    /// 2D counterpart of 3D's max_offset_surface_normal_deviation_at_vertex().
    double max_offset_normal_deviation_at_vertex(const size_t vid) const;

    /// Seed the sizing field from the offset's CURRENT edge lengths (paper Sec. 5.3.3, Step 1),
    /// once, before the first operation pass. Without it the field starts at the background
    /// target length and the first collapse pass decimates the offset.
    void init_offset_sizing_field();

    /// One {max_dist_err, avg_dist_err, max_norm_dev, avg_norm_dev} entry per optimization
    /// iteration actually run -- shorter than optimization_iterations when the loop converged
    /// and stopped early.
    std::vector<std::array<double, 4>> optimization_metrics;

    /// Accepted operations per optimization iteration: {splits, collapses, swaps}, one entry per
    /// iteration, in step with optimization_metrics. The counters are reset at the top of each
    /// iteration and read after that iteration's operation passes, so each entry is that
    /// iteration's own count, not a running total. Mirrors the 3D counters of the same names.
    std::vector<std::array<int, 3>> op_counts;
    std::atomic<int> iter_cnt_split = 0, iter_cnt_collapse = 0, iter_cnt_swap = 0;
    /// Collapses and swaps refused by the normal-deviation guards this iteration -- reported,
    /// not stored, because they say whether the guards are what is holding the offset's
    /// resolution up.
    std::atomic<int> iter_cnt_collapse_nd_reject = 0;
    std::atomic<int> iter_cnt_swap_nd_reject = 0;

    /// The offset surface's worst normal deviation around the collapsing edge, captured in
    /// collapse_edge_before() for collapse_edge_after() to compare against. Thread-local: the
    /// collapse pass runs in parallel, exactly as the 3D m_collapse_nd_before it mirrors.
    mutable wmtk::threading::enumerable_thread_specific<double> m_collapse_nd_before;

    /// The same for swap, plus the four vertices of the two incident triangles: after the flip
    /// the tuple names the NEW edge, so the two apexes would otherwise be unreachable.
    mutable wmtk::threading::enumerable_thread_specific<double> m_swap_nd_before;
    mutable wmtk::threading::enumerable_thread_specific<std::array<size_t, 4>> m_swap_verts;

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
    /// The offset's normal-deviation guard on collapse (paper Sec. 5.3.3, Step 2). Needs to be
    /// the bool-returning hook rather than collapse_after_vertex(), which cannot reject.
    bool collapse_edge_after(const Tuple& t) override;
    void split_after_vertex(size_t v_new) override;

    /**
     * @brief Carry each parent's region label onto the two children it became.
     *
     * This is bookkeeping, not positioning, and split_after_vertex() would be the natural home
     * for it. It lives here because of WHEN the base calls the two. Everything downstream of
     * the label reads it during the same split: the containment check inside the base's
     * split_edge_after() runs surface_segment_is_outside() on both new segments, which reaches
     * surface_envelope_for_edge() -> edge_is_region_boundary_live() -> face_is_offset_band(),
     * and that is a label test now. split_after_vertex() runs after that check; this hook is
     * the last one the base offers before it.
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

    /// Note the test is `== INPUT`, not `!= OFFSET`: a region boundary is neither, and the two
    /// are held to different rules -- see surface_envelope_for_edge().
    bool edge_is_input(const size_t eid) const
    {
        return m_edge_attribute[eid].m_is_surface_fs &&
               m_edge_attribute[eid].m_surface_class == INPUT_SURFACE_CLASS;
    }

    /**
     * @brief The one simplex set the optimization may not touch at all.
     *
     * The domain boundary is the box the background mesh lives in; there is nothing outside it
     * for a vertex to move into and nothing the optimizer gains by re-triangulating it, so it is
     * refused categorically.
     *
     * The INPUT COMPLEX used to be refused here too, and is not any more. It is the geometry the
     * offset distance is measured against, but "measured against" is a statement about
     * m_input_complex_bvh -- which holds the input as loaded and is never rebuilt -- not about
     * the mesh elements that happen to represent it. Freezing those elements bought nothing and
     * cost two things: the faces pinned between two frozen vertices could never reach
     * stop_energy, and a band vertex sitting on the complex could never be moved off it, which
     * pinned the loop's own convergence metric flat (measured in 3D: 119 of 544 band vertices,
     * the metric identical to six significant figures at every iteration).
     *
     * The input complex is now tracked exactly as TriWild tracks its input surface: held inside
     * m_envelope, re-projected onto it by the shared smoother, and topologically preserved by
     * substructure_link_condition(), which collapse_edge_before() already applies
     * unconditionally. See §2 of the design note in optimize_offset().
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
     * @brief Build the containment envelope over every tag-region boundary, once, before the
     * optimization starts.
     *
     * A region boundary is an edge whose two incident faces carry different tag sets -- the
     * offset band against the background mesh, the input complex against everything else, and
     * any other tagged region in the file. Exactly the rule SimWildMeshTri uses to build its
     * own 2D envelope, and the same SampleEnvelope 2D edge overload does the work, so nothing
     * about the envelope itself is reimplemented here.
     */
    void init_region_boundary_envelope();

    /**
     * @brief Build the same envelope from the INPUT mesh, before the offset exists.
     *
     * The band's tags REPLACE the ones already on a face rather than joining them
     * (`m_face_attribute[f].tags = new_tag`, everything but `protected_tags` dropped). So a
     * region the band grows through loses its own tag there, and its boundary curve is cut
     * off at whatever contour conservative growth happened to stop on. Built afterwards, the
     * envelope is a tube around that truncated curve, and its end cap pins the triple junction
     * where the region boundary meets the offset -- a vertex that should be free to slide along
     * its region's curve until it reaches target_distance, held instead at a position that is
     * an artefact of `relative_ball_threshold`.
     *
     * Built here, the tube follows the region's ORIGINAL, untruncated curve. That relaxes the
     * constraint in exactly one place -- along the stretch the band later swallows -- and
     * nowhere else, because a wider segment set can only ever make containment easier. No
     * vertex moves during offset construction (marching splits place the new vertex on the
     * edge being split), so the shared stretch of curve is geometrically identical either way.
     *
     * Requires only `label_input_complex()` to have run, for `face_is_input_complex()`. Inert
     * until `optimize_offset()`: nothing between here and there consults `m_envelope`, because
     * offset construction runs `TriMesh::split_edge` directly rather than the shared operations.
     */
    void init_region_boundary_envelope_from_input();

    /**
     * @brief Warn if the offset band has grown into the domain boundary.
     *
     * When target_distance exceeds the clearance between the input complex and the bounding
     * box, conservative growth runs out of room and the band's outer surface becomes the box
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

    /// Whether any tracked-surface segment at this vertex has left the envelope. The shared
    /// operations run their own containment checks through surface_envelope_for_edge(); this is
    /// for project_offset_vertex(), which places a vertex itself and never reaches them.
    bool region_boundary_is_outside_envelope(const size_t vid) const;

    /**
     * @brief Why smoothing refused a vertex, one counter per site where it can say no.
     *
     * The base's SmoothRejectCounters only sees the vertices that reach
     * TriOptimizerMesh::smooth_after(), which for the offset is the INTERIOR ones -- an
     * offset-boundary vertex is placed by project_offset_vertex() and never touches them. So
     * the whole offset boundary, the part that actually carries the distance error, is
     * invisible in the base's "accepted N | rejected: ..." line. These counters cover every
     * site on both paths.
     */
    struct SmoothTrace
    {
        std::atomic<int> attempted{0}; ///< smooth_before() entered
        std::atomic<int> before_bbox{0}; ///< base smooth_before said no: on the bounding box
        std::atomic<int> before_unrounded{0}; ///< base smooth_before said no: could not round
        std::atomic<int> before_on_input{0}; ///< input complex, frozen
        std::atomic<int> offset_attempted{0}; ///< dispatched to project_offset_vertex()
        std::atomic<int> offset_no_neighbours{0}; ///< no offset-boundary neighbour to average
        std::atomic<int> offset_on_complex{0}; ///< sits on the complex, no offset direction
        std::atomic<int> offset_inverted{0}; ///< search exhausted, a face still inverts
        std::atomic<int> offset_envelope{0}; ///< search exhausted, still outside the envelope
        std::atomic<int> offset_accepted{0};
        /// Accepted, but the binary search had to stop short of the blended target. These are
        /// invisible in offset_accepted, which is the point: a clamped move reports success
        /// while delivering a fraction of the requested correction.
        std::atomic<int> offset_clamped{0}; ///< accepted with the search fraction < 0.99
        std::atomic<int> offset_clamp_env{0}; ///< ... and the envelope is what stopped it
        std::atomic<int> offset_clamp_inv{0}; ///< ... and an inverted face is what stopped it
        /// ... and sliding along the region curve recovered ground the scaled search could not.
        /// A vertex flush against the envelope wall has a scaled search that can only return 0:
        /// every positive multiple of a vector pointing even slightly out of the tube is out of
        /// the tube. The tangential component of that same vector is legal and is most of it.
        std::atomic<int> offset_slid{0};
        /// Distance error over the offset vertices this pass actually touched, before and after
        /// the move, in units of 1e-9 so an atomic<int> can accumulate a max/sum.
        std::atomic<long long> offset_err_before_nano{0};
        std::atomic<long long> offset_err_after_nano{0};
        std::atomic<int> offset_err_max_before_nano{0};
        std::atomic<int> offset_err_max_after_nano{0};
        std::atomic<int> interior_attempted{0}; ///< dispatched to the shared AMIPS smoother
        std::atomic<int> region_attempted{0}; ///< ... of which sat on another region's boundary

        void reset()
        {
            offset_err_before_nano.store(0);
            offset_err_after_nano.store(0);
            for (std::atomic<int>* c :
                 {&attempted,
                  &before_bbox,
                  &before_unrounded,
                  &before_on_input,
                  &offset_attempted,
                  &offset_no_neighbours,
                  &offset_on_complex,
                  &offset_inverted,
                  &offset_envelope,
                  &offset_accepted,
                  &offset_clamped,
                  &offset_clamp_env,
                  &offset_clamp_inv,
                  &offset_slid,
                  &offset_err_max_before_nano,
                  &offset_err_max_after_nano,
                  &interior_attempted,
                  &region_attempted}) {
                c->store(0);
            }
        }
    };
    SmoothTrace m_smooth_trace;
    void log_smooth_trace() const;

    ////// wmtk::TriOptimizerMesh hooks

    /**
     * @brief REGION_SURFACE_CLASS and INPUT_SURFACE_CLASS segments carry a containment
     * requirement; the offset boundary does not.
     *
     * The envelope holds the other tag regions where they are, and -- since the complex stopped
     * being frozen -- the input complex too. That half is exactly TriWild's input envelope: the
     * complex may be split, collapsed and smoothed, and this is what bounds how far the result
     * may drift from the geometry as loaded.
     *
     * The offset boundary must be exempt: it is the surface the optimization exists to move, and
     * containing it inside a tube around its initial position caps how far it can ever travel
     * toward target_distance.
     *
     * Null means "no containment requirement", which the base handles by skipping the check.
     */
    std::shared_ptr<SampleEnvelope> surface_envelope_for_edge(
        const std::array<size_t, 2>& vids) const override
    {
        const auto [t, eid] = tuple_from_edge(vids);
        if (eid == static_cast<size_t>(-1) || !t.is_valid(*this)) {
            // The segment does not exist yet -- an operation asking about one it is about to
            // create. Answer with the envelope: containing a segment that turns out not to be a
            // region boundary only ever costs a rejected operation, while missing one that is
            // lets a region drift.
            return m_envelope;
        }
        // LIVE for the region class, not edge_is_region_boundary(eid): this is called from
        // inside the very split/collapse/swap passes that create the segment being asked about,
        // well before the next label_offset_boundary() would ever revisit its cached class. See
        // edge_is_region_boundary_live()'s comment for the collapse case that motivated this.
        // edge_is_input() needs no live form: it reads the surface class the shared operations
        // themselves propagate onto the segments they create.
        return (edge_is_region_boundary_live(t) || edge_is_input(eid)) ? m_envelope : nullptr;
    }

    /// No per-vertex positional constraint beyond the envelope. An offset-boundary vertex never
    /// reaches the base's smoother (smooth_after routes it to project_offset_vertex), and an
    /// input-complex vertex is held by m_envelope like any TriWild surface vertex.
    bool smoothing_position_is_allowed(size_t, const Vector2d&) const override { return true; }

    /**
     * @brief The loop's convergence metric, normalized so that 1.0 means "done".
     *
     * The max of the THREE criteria this optimization has to meet, each divided by its own
     * target, so mesh_improvement() stops exactly when all of them are met:
     *
     *   - max face AMIPS over stop_energy      -- TriWild's, via quality_rel()
     *   - max distance error over convergence_target, over the REACHABLE band (below)
     *   - average normal deviation over convergence_normal_deviation, dropped when that is <= 0
     *
     * The average returned alongside it is the same expression over the three averages, so both
     * numbers live on the same 1.0 scale. Nothing reads the average; it is logged.
     */
    std::tuple<double, double> optimization_quality_stats() override;
    double optimization_stop_metric() const override { return 1.; }

    /// Re-derive the tracked surfaces from the face labels, and log where each of the three
    /// criteria stands. See the base's declaration for why this needs a hook at all.
    void optimization_iteration_begin() override;

    /**
     * @brief The band's distance error, split by whether the optimizer can do anything about it.
     *
     * REACHABLE: a band vertex free to be placed at target_distance. PINNED: one that cannot be,
     * whatever the optimizer does -- it lies on the input complex, where the distance is 0 by
     * definition and the envelope keeps it, or on the domain boundary, where conservative growth
     * ran out of room. Neither is an optimization failure and neither can be improved by
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
    };
    DistanceSplit distance_deviation_split() const;
    /// Which vertices lie on the band's OUTER surface -- the one that is supposed to sit at
    /// target_distance. Shared by every measurement so they all agree on what "the band" is.
    std::vector<bool> band_vertex_mask() const;
    /// |dist(vid, input complex) - target_distance|.
    double band_vertex_distance_error(const size_t vid) const;
    /// Whether vid is a band vertex the optimizer could still place at target_distance.
    bool band_vertex_is_reachable(const size_t vid) const
    {
        return !m_vertex_extra[vid].m_is_on_input && !vertex_is_on_domain_boundary(vid);
    }

    /**
     * @brief TriWild's stall-driven sizing refinement, scored by this application's criterion.
     *
     * Structurally TriWildMesh::refine_sizing_around_worst, down to the shared helpers in
     * wmtk/utils/SizingField.hpp and every stuck_refine_* parameter: rank faces, force-split the
     * worst ones' longest edges, grow the region by rings, lower the per-vertex sizing scalar,
     * grade it outward.
     *
     * The one substitution is the per-face score. TriWild ranks a face by its AMIPS energy;
     * here a face is ranked by the max of the same three normalized criteria the loop stops on,
     * evaluated on that face. Ranking by AMIPS while the loop stalls on distance would refine
     * the wrong elements.
     */
    size_t refine_sizing_around_worst(double max_metric) override;
    /// The per-face score refine_sizing_around_worst ranks by; >= 1 means the face fails at
    /// least one criterion. Also the per-face form of optimization_quality_stats()'s max.
    double face_criterion_rel(const size_t fid) const;
    void write_smoothing_debug_output(const std::string& path) const override
    {
        const_cast<TopoOffsetTriMesh*>(this)->write_vtu(path);
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
     * @brief initialize BVH for input complex. Must be called after init_from_image(...)
     */
    void init_input_complex_bvh();

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
    void execute_offset(const std::filesystem::path& output_file);

    /**
     * @brief execute simplistic marching tris. All edges with one vertex labelled 0 and the other
     * 1/2 are split, at the midpoint (m_edge_split_mode=Midpoint) or at the hacky initialization
     * offset (m_edge_split_mode=Initial).
     *
     * There is no longer a distance-field mode here. Splitting the marched edge at the root of
     * d(l) - target_distance is not what the paper does and it is not what places the offset:
     * conservative growth decides where the boundary goes, and the optimization phase moves it
     * from there. See the note on the removal in .claude/CLAUDE.md.
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

    // variable offset stuff

    /**
     * @brief check if removing the given face from the given tag set would retain its topology
     */
    bool tag_tri_consistent_topology(size_t f_id, int64_t tag) const;

    /**
     * @brief check if adding a triangle to the offset region does not change the topology of the
     * offset. Returns true if topology would not be changed
     */
    bool offset_tri_consistent_topology(const size_t f_id) const;

    /**
     * @brief check if a triangle is inside the offset (implicitly defined via BVH distance field to
     * input complex) via conservative circle subdivision estimation
     */
    bool tri_is_in_offset_conservative(const size_t f_id, const double threshold_r) const;

    /**
     * @brief grow offset region conservatively using conservative checks while ensuring consistent
     * topology
     */
    void grow_offset_conservative();
    //// variable offset stuff

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
