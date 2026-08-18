#pragma once

#include <array>

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

    /// Refine the shared sizing field around the offset faces that are still over tolerance
    /// after Phase B converged. Returns the number of vertices whose scalar was lowered.
    size_t refine_sizing_where_phi_is_stuck();

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
     * @brief The input complex's OWN containment envelopes, one per stratum. Both phases.
     *
     * TWO of them because a SampleEnvelope is one kind or the other and never both: init() with
     * triangles builds Kind::Triangles3d, init() with segments builds Kind::Edges3d, and
     * is_outside(triangle) throws against the wrong one (require_exact_kind). The input complex is
     * an arbitrary closed simplicial complex, so it has geometry in both dimensions at once and
     * one envelope cannot hold it. Hence the pair, dispatched per simplex.
     *
     *   m_input_tri_env  Phi's triangles (m_phi_F): the boundary faces of the complex's tet
     *                    region plus its isolated triangles. Null when the complex has none --
     *                    a wire or a point cloud -- and every triangle query then falls through.
     *   m_input_seg_env  Phi's edges (m_phi_E) AND its isolated points (m_phi_P), the latter as
     *                    the degenerate segment (i, i). fast-envelope handles that explicitly:
     *                    halfspace_generation() tests AB == 0 and emits an axis-aligned cube of
     *                    half-width eps/sqrt(3) centred on the point instead of a hexahedron
     *                    around a direction it cannot normalise. SimpleBVH's
     *                    point_segment_squared_distance guards l2 == 0 the same way, so the
     *                    sampled path and nearest_point() agree with it.
     *
     * WHY THESE ARE NOT m_envelope. m_envelope is built from the mesh's region-boundary faces as
     * they stand AT CONSTRUCTION -- after marching tets and simplicial embedding have
     * re-triangulated the input. These are built from m_phi_*, the input AS LOADED, which is the
     * geometry m_input_complex_bvh and m_offset_potential measure against. Containing the input
     * complex within a tube around the input itself is the constraint the algorithm actually
     * needs; containing it within a tube around its own current mesh representation is not.
     *
     * INTERIOR VERTICES OF THE COMPLEX ARE NOT HELD BY THESE. Phi is the complex's BOUNDARY --
     * a face interior to the complex's tet region has two incident complex tets and is dropped by
     * the boundary_count filter in init_input_complex_bvh(). A tet-filled complex's interior
     * vertices are therefore unconstrained, which is what lets the mesh inside it be optimised.
     */
    std::shared_ptr<SampleEnvelope> m_input_tri_env;
    std::shared_ptr<SampleEnvelope> m_input_seg_env;

    /// Half-width of both envelopes above. Same knob as m_envelope_eps: the tolerance the input
    /// surface may drift within, which is what TetWild gives its own input surface.
    double m_input_envelope_eps = -1;

    /**
     * @brief Whether Phi has geometry BELOW its triangles -- isolated wires or isolated points.
     *
     * The discriminator for whether a per-vertex stratum test is needed at all. When Phi's only
     * edges are the edges of its triangles, every complex vertex is on the 2-stratum and
     * m_input_tri_env is the right answer for all of them; asking which stratum a vertex is on
     * would cost a one-ring walk per smoothing attempt to always get the same answer. A complex
     * WITH wires or points has vertices the triangle envelope would pull to the wrong place, and
     * vertices no tracked triangle covers at all, so there the walk earns its cost.
     *
     * Set from the extraction's own isolated-edge and isolated-vertex counts, which
     * init_input_complex_bvh() computes exactly (an edge is isolated iff it is in no face's or
     * tet's closure).
     */
    bool m_input_has_lower_strata = false;

    /// DIAGNOSTIC: how many times each envelope answered, and how many input-complex containment
    /// checks refused a move. Logged per phase; see log_input_envelope_trace().
    mutable std::atomic<size_t> m_input_env_tri_hits{0};
    mutable std::atomic<size_t> m_input_env_seg_hits{0};
    mutable std::atomic<size_t> m_input_env_point_refusals{0};

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

    /**
     * @brief Is this vertex INPUT-COMPLEX geometry, as opposed to merely region geometry.
     *
     * m_is_on_input is the live flag for it: init_surfaces_and_boundaries() sets it on every
     * two-sided region boundary, is_edge_on_input() propagates it at each split and collapse ORs
     * it onto the survivor. On a model whose complex bounds a tag region -- prism -- that set and
     * the region-boundary set coincide exactly; they part company on a complex with isolated
     * triangles, wires or points, which bound no region at all.
     *
     * NOT on the wall. A vertex on the bounding box is region geometry whatever else it carries,
     * and routing it to an input envelope it is nowhere near would refuse every operation on it --
     * which is precisely the degeneracy trap that freezing the wall used to manufacture. The flag
     * is documented as over-broad (see check_no_vertex_on_both_surfaces: a split propagates it and
     * a collapse ORs it, so a vertex can carry it while sitting a full target_distance away), and
     * this is the cheap half of guarding against that -- no BVH query, just the wall test the
     * dispatch already has to make.
     */
    bool vertex_is_input_geometry(const size_t vid) const
    {
        return m_vertex_extra[vid].m_is_on_input && m_vertex_attribute[vid].on_bbox_faces.empty();
    }

    std::shared_ptr<SampleEnvelope> surface_envelope_for_face(
        const std::array<size_t, 3>& vids) const override
    {
        bool all_region = true, all_offset = true, all_input = true;
        for (const size_t v : vids) {
            all_region = all_region && vertex_is_on_region(v);
            all_offset = all_offset && m_vertex_extra[v].m_is_on_offset;
            all_input = all_input && vertex_is_input_geometry(v);
        }
        // THE INPUT COMPLEX FIRST, in BOTH phases, because its envelope is the tighter statement
        // of the same constraint. m_envelope is a tube around the region boundary AS THE MESH
        // CONSTRUCTED IT; m_input_tri_env is a tube around the input as loaded, which is the
        // geometry m_offset_potential measures the offset against. Where the two coincide -- a
        // complex that bounds a tag region -- this moves the check from the second to the first,
        // and the difference is whatever marching tets and simplicial embedding did to the input.
        if (all_input && m_input_tri_env) {
            ++m_input_env_tri_hits;
            return m_input_tri_env;
        }
        // Region boundaries next, in BOTH phases: a face on both this and the offset carries
        // region geometry, which may not drift, so its envelope wins over the offset's looser
        // one. m_envelope holds every tag boundary AND the domain wall -- one envelope for the
        // whole partition, which is what makes the wall refinable instead of frozen.
        if (all_region) return m_envelope;
        // Phase A holds the offset where Phase B left it; Phase B is what moves it, so it is
        // unconstrained there. Null when there is no offset envelope yet -- the construction
        // phase runs before the first one is built.
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
     * The INPUT complex is the other tracked surface and gets TetWild's answer -- m_envelope, in
     * both roles -- because it is now smoothed like any TetWild surface vertex rather than
     * frozen. See smooth_before(): what pins the input geometry is that m_input_complex_bvh and
     * m_offset_potential are built once from the input as loaded, not that the mesh elements
     * representing it are immovable.
     */
    std::shared_ptr<SampleEnvelope> smoothing_energy_envelope(const size_t vid) const override
    {
        if (m_vertex_extra[vid].m_is_on_offset && !vertex_is_on_region(vid)) {
            return nullptr;
        }
        if (const auto env = input_complex_envelope_for_vertex(vid)) return env;
        return m_envelope;
    }

    /**
     * @brief Which of the input complex's two envelopes holds THIS vertex, or null.
     *
     * The per-simplex dispatch, for the PULL. nearest_point() is a BVH query and answers against
     * either kind, so the choice here is not about what the envelope can do -- it is about which
     * stratum of Phi the vertex actually belongs to. Pulling a wire vertex onto the nearest
     * TRIANGLE would drag it off its wire and toward geometry it was never on; pulling a
     * triangle-interior vertex onto the nearest SEGMENT would drag it to the triangle's rim,
     * since Phi is closed and every triangle edge is in m_phi_E.
     *
     * The stratum test costs a one-ring walk, so it only runs when Phi HAS a lower stratum for
     * the vertex to be on. With no wires and no isolated points every complex vertex is on a
     * triangle and the walk would always return the same answer -- see m_input_has_lower_strata.
     */
    std::shared_ptr<SampleEnvelope> input_complex_envelope_for_vertex(const size_t vid) const
    {
        if (!vertex_is_input_geometry(vid)) return nullptr;
        if (!m_input_has_lower_strata) {
            if (m_input_tri_env) ++m_input_env_tri_hits;
            return m_input_tri_env;
        }
        if (m_input_tri_env && vertex_has_tracked_input_face(vid)) {
            ++m_input_env_tri_hits;
            return m_input_tri_env;
        }
        if (m_input_seg_env) ++m_input_env_seg_hits;
        return m_input_seg_env;
    }

    /// Whether any tracked surface face at `vid` is input-complex geometry, i.e. whether this
    /// vertex is on Phi's 2-stratum. Defined out of line: it needs get_surface_faces_for_vertex().
    bool vertex_has_tracked_input_face(const size_t vid) const;

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
        // THE INPUT COMPLEX IS CONTAINED IN ITS OWN ENVELOPE, in both phases, for the same reason
        // it is in surface_envelope_for_face(): the tube that matters is the one around the input
        // as loaded. m_input_tri_env and not the pair, because what the caller does with this is
        // is_outside(triangle) over the vertex's tracked faces, and only a Triangles3d envelope
        // answers that -- a segment envelope throws (require_exact_kind). The 1- and 0-strata are
        // held instead by the POINT query in smooth_after(), which both kinds answer; a vertex on
        // a wire has no tracked triangle for this loop to test anyway.
        if (vertex_is_input_geometry(vid) && m_input_tri_env) return m_input_tri_env;
        // Region boundaries, the domain wall among them, are contained in BOTH phases: Phase B
        // frees the offset surface because that is the surface it exists to move, and the
        // partition is not it.
        return m_envelope;
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
    std::shared_ptr<polysolve::nonlinear::Problem> smoothing_extra_energy(
        const size_t vid) const override
    {
        // PHASE A CARRIES NO OFFSET TERM AT ALL. It is TetWild, and TetWild's smoothing
        // minimises AMIPS; the offset surface is held by m_offset_envelope there, not by an
        // energy. Phase B is the only pass that places it.
        if (m_phase != OptPhase::B) {
            return nullptr;
        }
        const auto& ve = m_vertex_extra[vid];
        if (!ve.m_is_on_offset || ve.m_is_on_input || !m_offset_potential) {
            return nullptr;
        }
        // Same weight the envelope term carries in the applications that have one, and for the
        // same reason: with w_amips at its default 1e-4 the placement dominates and AMIPS is a
        // light shape preference on top.
        return std::make_shared<OffsetEnergy3D>(m_offset_potential, m_params.w_envelope);
    }

    /// The driver hands a bare `debug_N`; put the frames beside the run's own output instead of
    /// in whatever directory it happened to be launched from. Same as 2D's
    /// write_smoothing_debug_output(), and what lets the viewer find them as a timeline -- it
    /// globs `*debug_*.vtu` in the output directory and orders them by the counter.
    void write_optimization_debug_output(const std::string& path) override
    {
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
    /// frozen there because conservative growth ran out of room. Neither is fixable.
    bool band_vertex_is_reachable(const size_t vid) const
    {
        return !m_vertex_extra[vid].m_is_on_input && m_vertex_attribute[vid].on_bbox_faces.empty();
    }

    /// Which vertices lie on the offset surface. Shared by every measurement so they all agree
    /// on what "the band" is.
    std::vector<bool> band_vertex_mask() const;

    /// The residual sampled at points INSIDE an offset-surface face -- see offset_face_samples().
    struct FaceSamples
    {
        double max = 0.;
        double sum = 0.;
        size_t n = 0;
    };

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
     * @brief The offset surface's residual, split by whether the optimizer can do anything about
     * it. See band_vertex_is_reachable(). Only the reachable half drives the loop and the sizing
     * field; the pinned half is reported so a construction defect stays visible.
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
     * @brief Build m_input_tri_env and m_input_seg_env from the extraction above.
     *
     * Called at the end of init_input_complex_bvh(), from the same m_phi_* it has just filled, so
     * the containment envelopes describe the same geometry as the distance field and the
     * potential. `n_isolated_edges` and `n_isolated_points` come from that extraction's own
     * counts and set m_input_has_lower_strata.
     */
    void init_input_complex_envelopes(int n_isolated_edges, int n_isolated_points);

    /// Log the per-phase input-envelope counters and reset them. See m_input_env_tri_hits.
    void log_input_envelope_trace(const char* when) const;


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
    /// The base rounds and refuses bbox vertices; this additionally freezes the input
    /// complex, and routes offset-surface vertices to the quadrics step below.
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;
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
        std::atomic<int> offset_attempted{0}; ///< reached the smoother carrying the offset term
        std::atomic<int> interior_attempted{0}; ///< reached it as an ordinary interior vertex

        void reset()
        {
            for (std::atomic<int>* c :
                 {&attempted,
                  &before_bbox,
                  &before_unrounded,
                  &before_on_input,
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
    std::vector<std::array<double, 4>> optimization_metrics;
    /// {splits, collapses, swaps} per optimization iteration, mirroring optimization_metrics.
    std::vector<std::array<int, 3>> op_counts;
    /// Whether the optimization met both convergence criteria before the iteration cap.
    bool m_converged = false;

    /// Per-iteration operation counters, reset before each iteration's operation passes.
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
