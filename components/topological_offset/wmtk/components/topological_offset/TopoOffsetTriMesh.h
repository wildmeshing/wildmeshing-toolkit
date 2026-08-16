#pragma once
#include <wmtk/TriMesh.h>
#include <wmtk/TriOptimizerMesh.h>
#include <wmtk/optimization/solver.hpp>
#include <algorithm>
#include <atomic>
#include <functional>
#include <set>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include "OffsetPotential.hpp"
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
     */
    SimplicialComplexBVH m_input_complex_bvh;

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
     * OFFSET is what sent the offset smoothing energy at vertices sitting ~40x target_distance
     * from the input complex, and what left the sizing field refining around them forever.
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
     * @brief Live region-boundary test for an edge that currently exists in the mesh.
     *
     * Unlike edge_is_region_boundary(), which reads m_edge_attribute[].m_surface_class -- a
     * snapshot label_offset_boundary() only refreshes once per optimization ITERATION -- this
     * recomputes the same rule label_offset_boundary() applies, straight from the current face
     * tags, every time it is called.
     *
     * This is what surface_envelope_for_edge() and smoothing_envelope() must use, not the
     * cached class: split, collapse and swap all run between one relabelling and
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

    /// Whether this edge is on the band's OUTER surface, recomputed from the tags on every call
    /// -- the live counterpart of edge_is_offset(), for use inside the operation passes.
    bool edge_is_offset_surface_live(const Tuple& e) const;

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
    /// Operations refused because they would have left an offset-boundary face over tolerance.
    /// The worst offset-face criterion around the operation, captured before it runs -- the bar
    /// its `after` hook compares against, mirroring how the AMIPS gates use cache.max_energy.
    wmtk::threading::enumerable_thread_specific<double> m_collapse_offset_rel_before;
    wmtk::threading::enumerable_thread_specific<double> m_swap_offset_rel_before;
    std::atomic<int> iter_cnt_collapse_offset_reject{0};
    std::atomic<int> iter_cnt_swap_offset_reject{0};
    /// Splits of an OFFSET-boundary edge: offered, accepted. The 3D twin showed splits landing
    /// everywhere BUT the offset surface, so this is what says whether 2D does the same.
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

    /**
     * @brief A vertex that IS the input complex may not leave it.
     *
     * For a 2- or 1-dimensional input the envelope already does this, and this is the same tube
     * evaluated pointwise. For a 0-DIMENSIONAL one there is no envelope at all: the envelope is
     * built from SEGMENTS where the tags or the input-ness change, and an input made of isolated
     * points has none, so init_region_boundary_envelope_from_input() finds nothing to contain
     * and every input point is left free to be smoothed like an ordinary interior vertex.
     *
     * It is not one. m_input_complex_bvh holds the input as LOADED and every offset quantity is
     * measured against it, so a mesh vertex that drifts off an input point takes the band with
     * it and leaves the offset trailing behind geometry that is no longer there. Measured on
     * topological_offset_2d_vertex_input: the three input points drifted, the band followed, and
     * the band's own boundary ended up 0.001 from the original point -- where Phi diverges, the
     * residual reads infinity and the run's convergence metric is meaningless.
     *
     * The rule admits exactly the motion TriWild admits for a surface vertex: free to slide
     * ALONG the complex (a vertex on an input segment stays at distance 0 wherever it moves on
     * it), pinned across it. An isolated point has nowhere to slide, so it is pinned outright.
     *
     * Everything an offset-boundary vertex has to satisfy is in the objective instead -- the
     * offset term -- or in the shared smoother's own accept checks.
     */
    bool smoothing_position_is_allowed(const size_t vid, const Vector2d& p) const override
    {
        if (!m_vertex_extra[vid].m_is_on_input) return true;
        return m_input_complex_bvh.dist(VectorXd(p)) <= m_offset_params.envelope_size;
    }

    /**
     * @brief The offset boundary is the one tracked surface with NO containment envelope.
     *
     * It is the surface the optimization exists to move: a tube around wherever conservative
     * growth left it would cap how far it can ever travel toward the level set. Every other
     * vertex -- input complex, another region's outline, plain interior -- gets the base's
     * answer unchanged.
     *
     * A vertex on the offset boundary AND on one of the others keeps the envelope: the region
     * curve it also belongs to still has to be held where it is, and the offset term is a
     * penalty rather than a hard constraint, so the two coexist.
     */
    std::shared_ptr<SampleEnvelope> smoothing_envelope(const size_t vid) const override
    {
        const auto& ve = m_vertex_extra[vid];
        if (ve.m_is_on_offset && !ve.m_is_on_input) {
            return nullptr; // EXPERIMENT: was `&& !ve.m_is_on_region`
        }
        return wmtk::TriOptimizerMesh::smoothing_envelope(vid);
    }

    /**
     * @brief The offset term, for a vertex on the offset boundary; null for everything else.
     *
     * THIS IS WHERE THE OFFSET IS PLACED. w (Phi - c)^2, minimised by the shared smoother
     * alongside AMIPS -- so the offset boundary is subject to the same line search, the same
     * exact inversion test and the same quality veto as every other vertex in the mesh, which
     * the previous hand-rolled projection was not.
     *
     * A vertex that is also on the INPUT COMPLEX is excluded. It sits at distance 0 from the
     * complex by definition, where Phi diverges; asking it to reach the level set would be
     * asking it to leave the geometry the offset is measured from. Those vertices are what
     * distance_deviation_split() counts as pinned.
     */
    std::shared_ptr<polysolve::nonlinear::Problem> smoothing_extra_energy(
        const size_t vid) const override
    {
        const auto& ve = m_vertex_extra[vid];
        if (!ve.m_is_on_offset || ve.m_is_on_input || !m_offset_potential) {
            return nullptr;
        }
        // Same weight the envelope term carries, and for the same reason: with w_amips at its
        // default 1e-4 the placement dominates and AMIPS is a light shape preference on top.
        return std::make_shared<OffsetEnergy2D>(m_offset_potential, m_params.w_envelope);
    }

    /**
     * @brief The loop's convergence metric, normalized so that 1.0 means "done".
     *
     * The max of the TWO criteria this optimization has to meet, each divided by its own
     * target, so mesh_improvement() stops exactly when both are met:
     *
     *   - max face AMIPS over stop_energy -- TriWild's, via quality_rel()
     *   - max Phi residual over offset_residual_rel * target_distance, over the REACHABLE band
     *
     * The average returned alongside it is the same expression over the two averages, so both
     * numbers live on the same 1.0 scale. Nothing reads the average; it is logged.
     */
    std::tuple<double, double> optimization_quality_stats() override;
    double optimization_stop_metric() const override { return 1.; }

    /// The two criteria, each normalized by its own target so 1.0 means met.
    struct Criteria
    {
        double amips = 0.;
        double phi = 0.;

        double max() const { return std::max(amips, phi); }
    };
    Criteria optimization_criteria();

    /// Samples per band edge; see offset_edge_samples(). 0 falls back to a vertex-only
    /// criterion, which is measurably blind to a band too coarse to be the offset.
    int offset_residual_samples() const { return m_offset_params.offset_residual_samples; }

    /// The tolerance the Phi residual is measured against: a fraction of target_distance, so
    /// "how close is close enough" is stated in units of the offset the run asked for.
    double offset_residual_tolerance() const
    {
        return std::max(m_offset_params.offset_residual_rel * m_offset_params.target_distance,
                        1e-16);
    }

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

    /**
     * @brief Refine only when EVERY criterion that is still unmet has stopped improving.
     *
     * The base asks the question of one scalar, and that scalar is the max -- so it only ever
     * reports on whichever criterion is currently worst, and a run whose worst criterion is
     * stuck refines on every iteration however well the other is doing. Measured on
     * topological_offset_2d: distance sat near 9.6x and was the max throughout, while AMIPS
     * moved between 0.78x and 1.87x and the normal deviation moved too; neither ever entered
     * the comparison, refinement fired on all 20 iterations, and the mesh grew from 2601 to
     * 69855 vertices.
     *
     * A criterion already at or below its target is excluded rather than counted as stalled:
     * it is finished, so it is neither a reason to refine nor a reason to hold off. The base's
     * own inequality is then applied to each of the rest -- both are normalized to a target of
     * 1.0, so it is the identical formula with no new parameter.
     */
    bool optimization_stalled(double prev, double cur) override;

    /**
     * @brief Consolidate at every pass boundary, whether or not a debug frame is written.
     *
     * write_vtu() consolidates, which renumbers, which changes the order every later pass
     * enumerates operations in -- so with the write gated on debug_output, turning the flag on
     * silently produced a different run. It was measured on the dragon rectangle as
     * converged-in-7 against not-converged-in-10. 3D settled this by consolidating whether or
     * not anything is written; this is the same remedy at the pass boundary, which is where the
     * shared driver writes its frames.
     */
    void optimization_debug_checkpoint() override;

    /// Re-derive the tracked surfaces from the face labels, and log where each of the two
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
        /// max_reachable, split by where it was measured. The two answer different questions:
        /// the vertex max says the boundary is in the wrong PLACE, the edge max says it is too
        /// COARSE to be in the right place -- and they call for different remedies (smoothing vs
        /// refinement), so a run that is not converging needs to know which it is. On the dragon
        /// the whole residual is in the second: 0.73% of delta at the vertices, 14% between them.
        double max_at_vertex = 0., max_in_edge = 0.;
    };
    DistanceSplit distance_deviation_split() const;

    /// The same split over the quantity the LOOP converges on: the Phi residual, as a length.
    /// Reported beside the Euclidean one so the two offsets can always be compared.
    DistanceSplit residual_split() const;
    /// Which vertices lie on the band's OUTER surface -- the one that is supposed to sit at
    /// target_distance. Shared by every measurement so they all agree on what "the band" is.
    std::vector<bool> band_vertex_mask() const;
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
     * here a face is ranked by the max of the same two normalized criteria the loop stops on,
     * evaluated on that face. Ranking by AMIPS while the loop stalls on the offset would refine
     * the wrong elements.
     */
    size_t refine_sizing_around_worst(double max_metric) override;

    /// No. The offset boundary is the thing being placed and has no envelope holding it, so a
    /// bare collapse pass -- one not interleaved with splits and smoothing, and answering to no
    /// criterion -- can only decimate it. See the base for the measurement (3D, but the same
    /// mechanism: measured there at 1172 offset faces down to 326 before the loop had run).
    bool optimization_bare_coarsen_passes() const override { return false; }

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
    /// The per-face score refine_sizing_around_worst ranks by; >= 1 means the face fails at
    /// least one of the two criteria. Also the per-face form of optimization_quality_stats().
    double face_criterion_rel(const size_t fid) const;
    /// The driver hands a bare `debug_N`; put the frames beside the run's own output instead of
    /// in whatever directory it happened to be launched from.
    void write_smoothing_debug_output(const std::string& path) const override
    {
        const_cast<TopoOffsetTriMesh*>(this)->write_vtu(m_offset_params.output_path + "_" + path);
    }

    /// Previous iteration's criteria, captured at the top of each iteration -- which is the end
    /// of the one before, the same interval the driver's own stall test spans.
    Criteria m_prev_criteria;


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
