#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>

#include <wmtk/TetMesh.h>
#include <wmtk/TetOptimizerMesh.h>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include "OffsetPotential.hpp"
#include "Parameters.h"
#include "SimplicialComplexBVH.hpp"
#include "TagEnvelopes.hpp"

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
 * @brief Per-vertex data the shared 3D optimizer knows nothing about.
 *
 * Position, rounding, bbox membership, order, sizing and partition live on
 * wmtk::TetOptimizerMesh::VertexAttributes. The three flags here say which tracked surface a
 * vertex belongs to; the base's m_is_on_surface is their union, and they are not exclusive -- a
 * vertex where the offset surface meets another region's boundary carries both. The 2D twin is
 * VertexExtra2d.
 */
class VertexExtra
{
public:
    int label = 0;
    size_t component_id = 0;
    bool m_is_on_input = false; // on the input complex
    bool m_is_on_offset = false; // on the offset surface itself
    bool m_is_on_region = false; // on some OTHER tag region's boundary
    /// Where this vertex stood at the start of the turn, so the convergence states can read its
    /// net movement. A split copies it on, so a new vertex reads as moved for the turn it was
    /// born in.
    Vector3d m_turn_start = Vector3d::Zero();
    bool m_turn_start_valid = false;

    /**
     * @brief Which tag boundaries this vertex lies on -- one bit per input tag, ambient included.
     * See TopoOffsetTetMesh::m_tag_envelopes for what the bits dispatch to.
     *
     * Seeded in init_surfaces_and_boundaries() from the input partition, then propagated by the
     * operations: a split's new vertex takes the AND of its endpoints (it lies on a boundary only
     * if the whole edge did), a collapse's survivor the OR (it carries both vertices' geometry).
     */
    uint64_t m_boundary_mask = 0;

    /// Churn instrumentation: which split pass created this vertex, from
    /// wmtk::TetOptimizerMesh::m_op_epoch; 0 means not created by an optimization split. Read
    /// only by collapse_after_vertex(). Assigned at each split, never OR'd -- a recycled slot
    /// carries a dead vertex's epoch.
    uint32_t m_born_epoch = 0;
};


class EdgeAttributes
{
public:
    int label = 0; // label: 0=default, 1=input, 2=offset
};


/// Per-face construction label; the surface tags themselves are the base's
/// wmtk::SurfaceTagAttributes. Registered with m_face_attr_group.
class FaceExtra
{
public:
    int label = 0; // label: 0=default, 1=input, 2=offset
    /// The face lies on the input's envelope surface group (within its envelope): selectable as
    /// the complex by name, held in that group's tube. Re-derived by classify_sheet_faces();
    /// nothing propagates it through split or collapse, so it is read only before the
    /// optimization starts. The 2D twin is EdgeExtra2d::on_curve.
    bool on_sheet = false;
};


class TetAttributes
{
public:
    int label = 0; // label: 0=default, 1=input, 2=offset
    CellTag tag;
    double m_quality = 0; // AMIPS energy, kept up to date by smoothing
    /**
     * Rest shape (deform_others): the tet's corners when it last changed topologically, in the
     * oriented order. Stamped for every deformable cell at release and re-stamped by the
     * operation after-hooks for every cell an accepted split / collapse / swap changed, never by
     * smoothing -- a child left on its parent's rest reads det F ~ 1/2 and fights to regrow.
     */
    bool rest_valid = false;
    std::array<Vector3d, 4> rest_pos;
};


/**
 * @brief The offset's tet mesh, on the shared 3D optimizer.
 *
 * Mirrors TopoOffsetTriMesh one dimension up: the construction phase (simplicial embedding,
 * marching tets, growing the band) is entirely its own, and the optimization phase that follows
 * is wmtk::TetOptimizerMesh's, with the offset supplying only policy through the hooks.
 *
 * Two surfaces are tracked. Every tag-region boundary (input complex and domain wall included)
 * keeps the primary class 0 and is held in its tags' envelopes, as tetwild holds its input; the
 * offset surface is OFFSET_SURFACE_CLASS, the faces across which the incident tet labels differ.
 * Class-0 faces may move within their tubes; only the offset one is driven toward
 * target_distance.
 */
class TopoOffsetTetMesh : public wmtk::TetOptimizerMesh
{
public: // mode for splitting in marching tets
    enum class EdgeSplitMode {
        Midpoint = 0, // construction: simplicial embedding AND marching_tets
        Optimization = 5 // the optimization phase; the shared engine places the vertex
    };

public:
    int m_vtu_counter = 0;
    std::array<size_t, 4> m_init_counts = {{0, 0, 0, 0}};
    size_t m_tags_count;
    /// Tag id of the input's envelope surface group (the .msh triangle elements), or -1. An open
    /// sheet has no tet set whose boundary it is, so it is selectable only through this tag:
    /// offset_selection naming it makes the sheet the complex and the band grows on both of its
    /// sides. The 2D twin is m_curve_tag.
    int64_t m_sheet_tag = -1;
    /// The surface group as loaded, kept because the classification below is redone on demand.
    MatrixXd m_sheet_V;
    MatrixXi m_sheet_F;
    /**
     * @brief Mark the mesh faces that lie on the input's envelope surface group
     * (FaceExtra::on_sheet).
     *
     * Geometric, against the sheet's own tube (the same eps the tag envelopes use), because the
     * .msh carries the sheet with its own vertices and there is no index to match on. Called
     * whenever the complex is labelled, not once at load: the flag is a property of a face and
     * nothing propagates it through split and collapse.
     */
    void classify_sheet_faces();
    /**
     * @brief The input complex as loaded. Built once, never rebuilt.
     *
     * It answers the Euclidean distance to the input, a diagnostic rather than the definition of
     * the offset -- see m_offset_potential. init_input_complex_bvh() has one call site, before
     * execute_offset() runs, so this holds the original geometry however the elements
     * representing the complex are later remeshed. Rebuilding from the live mesh would redefine
     * the offset distance in terms of a surface the optimizer had just moved.
     *
     * Containment is not its job -- the per-tag region envelopes (m_tag_envelopes) hold the
     * complex in place.
     */
    std::shared_ptr<SimplicialComplexBVH> m_input_complex_bvh;

    /**
     * @brief The smooth offset potential, and with it the definition of the offset itself.
     *
     * The offset surface is the level set Phi = c. Built from the same extraction as
     * m_input_complex_bvh, so the two describe the same geometry and the same never-rebuilt rule
     * applies. See OffsetPotential for what Phi is. shared_ptr because OffsetEnergy3D holds one
     * per smoothing call.
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
     * @brief One field per connected piece of the input complex, and which one each band vertex
     * is placed on. See TopoOffsetTriMesh::m_region_potentials for the full argument.
     *
     * m_offset_potential is built over the whole selected complex; where two pieces are close
     * the union field has no level set across the gap. A band grown from one piece is placed on
     * that piece's field alone. Pieces are the connected components of the captured complex under
     * vertex connectivity, numbered once in init_input_complex_bvh(); assign_band_regions() maps
     * the band's cells and vertices to them by a flood fill seeded from complex vertices.
     */
    int m_n_regions = 0; ///< connected pieces of the input complex; one field each
    std::vector<std::shared_ptr<OffsetPotential3D>> m_region_potentials; ///< one per piece
    /// One BVH per piece, over that piece's primitives alone: what assign_band_regions() reads
    /// a seed vertex's piece off (nearest piece), and the euclidean per-piece field's engine. 2D
    /// answers the same question through its BVH's feature ids, which the 3D BVH has none of.
    std::vector<std::shared_ptr<SimplicialComplexBVH>> m_region_bvhs;
    std::vector<int64_t> m_phi_vert_region; ///< per m_phi_V row: region index
    std::vector<int64_t> m_phi_seg_region; ///< per m_phi_E row: region index, -1 unknown
    std::vector<int64_t> m_phi_face_region; ///< per m_phi_F row: region index, -1 unknown
    std::vector<int64_t> m_phi_point_region; ///< per m_phi_P entry: region index, -1 unknown
    std::vector<int> m_cell_region; ///< per tet: band's region, -1 none, -2 reached from two
    std::vector<int> m_vertex_region; ///< per vertex: region of its band cells, -1 / -2 as above
    void init_region_potentials(double delta, double effective_factor);
    void assign_band_regions();
    /// Diagnostic: the front objective of one vertex along its normal, offset term vs total.
    void log_front_profile(size_t vid);
    int vertex_region(const size_t vid) const
    {
        return vid < m_vertex_region.size() ? m_vertex_region[vid] : -1;
    }
    int edge_region(const size_t va, const size_t vb) const
    {
        const int a = vertex_region(va), b = vertex_region(vb);
        return (a >= 0 && a == b) ? a : -1;
    }
    const OffsetPotential3D& potential_for_region(const int region) const
    {
        return (region >= 0 && size_t(region) < m_region_potentials.size())
                   ? *m_region_potentials[size_t(region)]
                   : *m_offset_potential;
    }
    const OffsetPotential3D& potential_for(const size_t vid) const
    {
        return potential_for_region(vertex_region(vid));
    }
    /// The same selection as potential_for(), as the pointer the energies take a share of. Null
    /// only when m_offset_potential is, which the front placement paths test for.
    std::shared_ptr<const OffsetPotential3D> potential_ptr_for(const size_t vid) const
    {
        const int r = vertex_region(vid);
        return (r >= 0 && size_t(r) < m_region_potentials.size()) ? m_region_potentials[size_t(r)]
                                                                  : m_offset_potential;
    }
    const OffsetPotential3D& potential_for_edge(const size_t va, const size_t vb) const
    {
        return potential_for_region(edge_region(va, vb));
    }
    /// The field of the band cell a live offset face belongs to.
    const OffsetPotential3D& potential_for_face(const Tuple& f) const;

    /**
     * @brief One containment envelope per input tag, ambient included. Both phases.
     *
     * E_t is a tube of half-width m_envelope_eps around region t's boundary faces as the input
     * mesh carried them, built in init_surfaces_and_boundaries() before offset construction: the
     * band's tags replace a tet's own, so an envelope built later would be a tube around a
     * surface truncated at the band. A simplex on several boundaries is held by the intersection
     * of its tags' tubes (envelope_for_mask()), which pins junction curves and points.
     *
     * m_envelope (the base's pointer) survives as a UnionEnvelope over these members, purely so
     * the shared engine's direct uses of it keep union semantics.
     */
    std::map<int64_t, std::shared_ptr<SampleEnvelope>> m_tag_envelopes;

    /// Input tag id -> bit position in VertexExtra::m_boundary_mask. Assigned in
    /// init_from_image() once the tag maps are complete; at most 64 input tags.
    std::map<int64_t, int> m_tag_bit;

    /// Memoized IntersectionEnvelope per multi-bit mask. Lazily built under the mutex because
    /// the queries that need them run concurrently under kPartition.
    mutable std::map<uint64_t, std::shared_ptr<SampleEnvelope>> m_isect_cache;
    mutable std::mutex m_isect_mutex;

    /**
     * @brief Memoized "region tubes AND the offset envelope", keyed by the region mask.
     *
     * Separate from m_isect_cache because the members differ in lifetime: the tag envelopes live
     * for the whole run, m_offset_envelope is rebuilt after every smoothing pass.
     * rebuild_offset_envelope() clears this and must keep doing so. Guarded by m_isect_mutex.
     */
    mutable std::map<uint64_t, std::shared_ptr<SampleEnvelope>> m_offset_isect_cache;

    /**
     * @brief The containment a simplex with this region mask, on/off the offset surface, must
     * satisfy -- the intersection of everything that holds it, or null if nothing does.
     *
     * The single place the two containment families are composed. `region_mask` dispatches
     * through envelope_for_mask(); `on_offset` adds m_offset_envelope, but only in Phase A --
     * the phases that place the front are what move the offset surface, so there the result is
     * the region tubes alone.
     */
    std::shared_ptr<SampleEnvelope> containment_for(uint64_t region_mask, bool on_offset) const;

    /**
     * @brief Move `x` back inside every region tube this vertex lies on. True if it ended up
     * inside all of them. Alternating projection onto the worst-violated real member; never asks
     * a composite (see TagEnvelopes.hpp).
     */
    bool project_into_containment(size_t vid, Vector3d& x) const;

    /**
     * @brief Which half of the alternating optimization is running. The 3D copy of
     * TopoOffsetTriMesh::OptPhase.
     *
     * Phase A is TetWild and nothing else: same operations, gates, sizing field and
     * stall-driven refinement, with no offset energy term. Its one addition is m_offset_envelope.
     * Phase B moves the offset surface and nothing else: smoothing against the offset energy.
     * Single is the mode the run uses: TetWild's operation groups with the front placed by Phase
     * B's objective inside the smoothing passes. It follows B wherever the smoother is concerned
     * (which objective a front vertex gets, no offset tube while it moves) and A everywhere the
     * loop is concerned (quality stats and the stop metric are TetWild's).
     */
    enum class OptPhase { A, B, Single };

    /// Whether the smoother places front vertices against the offset objective: Phase B, and
    /// the single-phase mode that does the same thing inside TetWild's passes.
    bool phase_places_front() const { return m_phase != OptPhase::A; }

    /// Which phase is running. Read by every hook that differs between them; see OptPhase.
    OptPhase m_phase = OptPhase::A;

    /// The final Phase A: front vertices are not smoothed (see smooth_before()).
    bool m_freeze_front = false;

    /**
     * @brief The tube the offset surface may not leave during the operation passes, of
     * half-width offset_envelope_rel x target_distance. Rebuilt after every smoothing pass from
     * the surface as that pass left it, which is what lets the surface travel across turns.
     * Non-null once the offset exists; whether it constrains is containment_for()'s phase test.
     * Unlike m_tag_envelopes, which must never be rebuilt.
     */
    std::shared_ptr<SampleEnvelope> m_offset_envelope;

    /// Rebuild m_offset_envelope from the current offset-surface faces, and drop the
    /// intersections memoized against the old one.
    void rebuild_offset_envelope();

    /// Hard error if any vertex is on both the input complex and the offset surface -- a state
    /// no placement satisfies. Called at construction and after every phase.
    void check_no_vertex_on_both_surfaces(const char* when) const;

    /// TetWild's loop, the front placed inside its smoothing passes.
    void optimize_offset_single_phase();

    /// Max over the front vertices of the vertex convergence measure (a ratio to its bar); under
    /// gradient_norm_rel and before the reference exists, the raw |n . grad F|. The pass stop.
    double phase_b_front_gradient_linf();
    /// Its value on the band as constructed, measured once before turn 1: the reference the
    /// gradient_norm_rel criterion is a fraction of.
    double m_front_gradient_reference = 0.;

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
    // m_envelope itself lives on the base, which is what checks tracked-surface triangles
    // against it.
    double m_envelope_eps = -1;

    /**
     * @brief SurfaceTagAttributes::m_surface_class: which of the two tracked surfaces a face
     * belongs to. Same scheme as 2D.
     *
     * OFFSET is the surface the optimization places at target_distance. Everything else -- the
     * input complex, another body's boundary, the domain wall -- keeps the primary class 0 and
     * is envelope-checked by the shared operations exactly as in tetwild and simwild. Class 0 is
     * not split further -- the boundary mask says which tubes hold a simplex, per tag.
     */
    static constexpr int INPUT_SURFACE_CLASS = 0;
    static constexpr int OFFSET_SURFACE_CLASS = 1;

    /// The base holds only wmtk::OptimizerParameters; this is the same object, typed.
    Parameters& m_offset_params;

    using VertexExtraCol = wmtk::AttributeCollection<VertexExtra>;
    using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes>;
    using FaceExtraCol = wmtk::AttributeCollection<FaceExtra>;
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    // m_vertex_attribute and m_face_attribute are the base's; these are registered alongside
    // them in its attribute groups.
    VertexExtraCol m_vertex_extra;
    FaceExtraCol m_face_extra;
    EdgeAttCol m_edge_attribute;
    TetAttCol m_tet_attribute;

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

        // As in 2D. The per-vertex Newton solver logs a line per smoothing attempt at info level,
        // which is one line per vertex per pass and buries the run's own output.
        optimization::deactivate_opt_logger();
    }

    ~TopoOffsetTetMesh() override = default;

    ////// wmtk::TetOptimizerMesh hooks

    double cell_quality(const size_t tid) const override { return m_tet_attribute[tid].m_quality; }
    void set_cell_quality(const size_t tid, const double q) override
    {
        m_tet_attribute[tid].m_quality = q;
    }

    /**
     * @brief Place a vertex, keeping its exact and rounded coordinates in step.
     *
     * The offset works in doubles throughout, so every vertex it places is rounded, but m_pos
     * must still be filled: the shared split's exact-midpoint fallback reads it, and every
     * quality and orientation test around an unrounded vertex reads its neighbours' m_pos.
     */
    void set_vertex_position(const size_t vid, const Vector3d& p)
    {
        m_vertex_attribute[vid].m_posf = p;
        m_vertex_attribute[vid].m_pos = to_rational(p);
        m_vertex_attribute[vid].m_is_rounded = true;
    }

    /// Whether face `fid` is on the offset surface / bounds a region -- any tracked face that is
    /// not the offset surface. The input complex is included, and deliberately: both are held by
    /// the same per-tag envelopes and neither is what the optimization moves.
    bool face_is_offset(const size_t fid) const
    {
        return m_face_attribute[fid].m_is_surface_fs &&
               m_face_attribute[fid].m_surface_class == OFFSET_SURFACE_CLASS;
    }
    bool face_is_region(const size_t fid) const
    {
        return m_face_attribute[fid].m_is_surface_fs &&
               m_face_attribute[fid].m_surface_class != OFFSET_SURFACE_CLASS;
    }

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
     * @brief Tag the two tracked surfaces for the optimization phase.
     *
     * The offset surface is the faces across which the incident tet labels differ, so it falls
     * out of the labelling and is recomputed here once. The 2D twin is
     * TopoOffsetTriMesh::label_offset_boundary().
     */
    void label_offset_boundary();

    /// Whether tet `tid` belongs to the closed offset region, read from its label: the band
    /// (label 2) plus the input complex it wraps (label 1). Every operation carries the label
    /// onto the cells it creates, so this is exact; tags cannot express the distinction.
    bool cell_in_region(const size_t tid) const
    {
        const int l = m_tet_attribute[tid].label;
        return l == 1 || l == 2;
    }
    /// Whether tet `tid` is part of the INPUT complex the band wraps.
    bool cell_is_input_complex(const size_t tid) const { return m_tet_attribute[tid].label == 1; }
    /// Whether tet `tid` is part of the offset BAND (as opposed to the input complex).
    bool cell_is_offset_band(const size_t tid) const { return m_tet_attribute[tid].label == 2; }

    /// The 3D optimization phase: split / collapse / swap / smooth on the shared driver.
    void optimize_offset(const std::filesystem::path& output_file);

    /**
     * @brief How far the offset surface is from where it should be: {max, avg} over vertices.
     *
     * The absolute error |dist(v, input complex) - target_distance| over the offset-surface
     * vertices only, pinned ones included. Mirrors TopoOffsetTriMesh::compute_distance_deviation().
     */
    std::pair<double, double> compute_distance_deviation() const;

    /// The vertex compute_distance_deviation() last found the max at, and a dump of everything
    /// that could be stopping it from moving. Diagnostic only.
    mutable size_t m_worst_dist_vid = static_cast<size_t>(-1);
    void log_worst_dist_vertex() const;

    /**
     * @brief The band's outer surface, recomputed live rather than read from the cached class.
     *
     * A band cell meeting a cell that is neither band nor input complex. Must be live: the
     * operations that ask run between one labelling pass and the next. Returns true for a band
     * face on the domain boundary, whose vertices are pinned and must be measured, not hidden.
     * The 2D twin is edge_is_offset_surface_live().
     */
    bool face_is_offset_surface_live(const Tuple& f) const;
    /// Whether edge (a, b) lies on the band's outer surface: some incident face does.
    bool edge_is_offset_surface_live(size_t a, size_t b) const;
    /// Every edge of the live offset surface, once. What the chord test and the alignment
    /// term enumerate; the 2D twin walks get_edges() and asks edge_is_offset_surface_live().
    std::vector<std::array<size_t, 2>> offset_surface_edges() const;
    /// The live offset-surface faces incident to vid.
    std::vector<Tuple> offset_surface_faces_live_at(size_t vid) const;

    /// Seed the sizing field from the offset's current edge lengths (paper Sec. 5.3.3, Step 1),
    /// once, before the first operation pass, then tighten the front's resolution from the
    /// tolerance and the level set's curvature.
    void init_offset_sizing_field();

    /// {max_dist_err, avg_dist_err, max_phi_residual, avg_phi_residual, max_grad, avg_grad,
    /// max_grad_at_vertex, max_grad_in_face}. One entry for the whole run, as in 2D.
    std::vector<std::array<double, 8>> optimization_metrics;
    /// {split-born vertices, recollapsed, recollapsed in the immediately following collapse
    /// pass} per turn, in step with op_counts. See VertexExtra::m_born_epoch.
    std::vector<std::array<int, 3>> churn_counts;
    /// {splits, collapses, swaps} per turn, as deltas rather than running totals.
    std::vector<std::array<int, 3>> op_counts;
    /// The turn the run is in, 1-based; 0 before the loop starts. Read only by
    /// write_optimization_debug_output(), to tag each frame with the turn it belongs to.
    int m_ab_round = 0;
    /// Monotonic frame counter for the debug timeline.
    mutable size_t m_debug_seq = 0;
    /// Pass index within the current phase, and the (round, phase) it belongs to -- when those
    /// change the index restarts. All three exist only to name frames.
    mutable int m_debug_pass = 0;
    mutable int m_debug_last_round = -1;
    mutable char m_debug_last_phase = '?';
    /// See offset_gradient_tolerance(). Nothing sets it on the single-phase path; it stays 0.
    double m_gradient_reference = 0.;
    bool m_converged = false;

    /// Churn: split-born vertices that a collapse later removed, and the subset removed in the
    /// same pass-pair that created them.
    std::atomic<int> iter_cnt_split_born{0};
    std::atomic<int> iter_cnt_recollapsed{0};
    std::atomic<int> iter_cnt_recollapsed_same_pass{0};
    std::atomic<int> iter_cnt_split = 0, iter_cnt_collapse = 0, iter_cnt_swap = 0;
    std::atomic<int> iter_cnt_collapse_offset_removed{0};
    /// Operations refused because they would have left an offset-surface face over tolerance.
    std::atomic<int> iter_cnt_collapse_offset_reject{0};
    std::atomic<int> iter_cnt_swap_offset_reject{0};
    /// Splits of an offset-surface edge: offered, accepted.
    std::atomic<int> iter_cnt_split_offset_before{0};
    std::atomic<int> iter_cnt_split_offset{0};

    /// What the shared split has to carry across for the offset: the region tag of each parent
    /// tet, keyed by the edge opposite the split one, and which surfaces the edge was on.
    struct OptSplitCache
    {
        bool is_edge_on_region = false;
        bool is_edge_on_offset = false;
        std::map<simplex::Edge, TetAttributes> tets;
        /// Diagnostic: the parents' worst AMIPS before the split, so split_after_vertex() can
        /// say whether a needle child came from a healthy parent or an already unscoreable one.
        double parent_q_max = -1.;
        /// Same question in the scale-invariant measure, which keeps resolving after AMIPS has
        /// saturated at MAX_ENERGY. Min over the parents: the flattest thing the split inherited.
        double parent_flatness = 1.;
    };
    wmtk::threading::enumerable_thread_specific<OptSplitCache> m_opt_split_cache;

    bool marching_split_edge_before(const Tuple& t);
    bool marching_split_edge_after(const Tuple& t);

    /**
     * @brief Reject any collapse that violates the substructure link condition, and remember the
     * survivor's sizing for sizing_collapse_min = false.
     *
     * The base applies the link condition only when both endpoints already sit on a tracked
     * surface or the bbox; the offset region is a thin shell, so a collapse with one endpoint in
     * the interior can still pinch its two sides together. The offset asks unconditionally.
     */
    bool collapse_edge_before(const Tuple& t) override;
    /// The coarsening bar, the sizing restore and the rest re-stamp, after the base accepted.
    bool collapse_edge_after(const Tuple& t) override;
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
     * @brief Which tag the tets a swap creates should carry, and the topology half of the
     * surface-flip refusal (class match, mask match). The geometric half is the shared swap's
     * containment check. See Swap.cpp.
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
     * @brief Split policy that is the offset's own: which region tag the two child tets inherit,
     * and which of the two tracked surfaces the new vertex joined. See EdgeSplittingTet.cpp.
     */
    bool split_before_cells(const Tuple& edge, const std::vector<Tuple>& parents) override;
    bool split_after_cells(size_t v1, size_t v2, size_t v_new, const std::vector<Tuple>& children)
        override;
    bool split_adjust_position(size_t v_new, const std::vector<Tuple>& children) override;
    void split_after_vertex(size_t v_new, bool is_edge_open_boundary) override;

    /// The offset's surface can end on a non-manifold or boundary edge of the input complex,
    /// which the base must not flip or split across.
    bool is_open_boundary_edge(const Tuple& e) override { return is_order_2_edge(e); }

    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    /**
     * @brief Identification only -- no operation refuses the domain wall through these.
     *
     * The wall is a tracked region boundary like every other one: init_surfaces_and_boundaries()
     * tags its faces m_is_surface_fs, masks its vertices with ambient's bit and puts its faces in
     * ambient's envelope, so refinement, coarsening, flips and smoothing are governed by the
     * same containment, merge rules and link conditions that govern the input complex. As in 2D.
     */
    bool vertex_is_on_domain_boundary(const size_t vid) const
    {
        return !m_vertex_attribute[vid].on_bbox_faces.empty();
    }
    bool face_is_on_domain_boundary(const size_t fid) const
    {
        return m_face_attribute[fid].m_is_bbox_fs >= 0;
    }

    /**
     * @brief Classify every region boundary, build the per-tag containment envelopes, and tag the
     * domain wall -- once, from the input mesh, before offset construction runs.
     *
     * A region boundary is a face whose two incident tets carry different tag sets; it enters
     * the bucket of every tag on exactly one side (the symmetric difference). A face with only
     * one incident tet is the domain wall and enters its single tet's tags' buckets, which is
     * how ambient's envelope comes to hold the box.
     */
    void init_surfaces_and_boundaries();

    /// Whether edge `loc` lies on a region boundary / on the offset surface, by the cached face
    /// classes.
    bool is_edge_on_region(const Tuple& loc);
    bool is_edge_on_offset(const Tuple& loc);

    /// Set VertexExtra::m_is_on_input from the construction labels, once label_input_complex()
    /// has evaluated the selection. The 2D twin has the same name.
    void mark_input_complex_vertices();

    /**
     * @brief Warn if the offset band has grown into the domain boundary.
     *
     * When target_distance exceeds the clearance between the input complex and the bounding box,
     * construction runs out of room and the band's outer surface becomes the box itself; those
     * vertices are pinned and the target distance is unreachable there.
     */
    void warn_if_offset_reaches_domain_boundary() const;

    /**
     * @brief What smoothing did with each class of vertex, per pass. Same fields as 2D.
     */
    struct SmoothTrace
    {
        std::atomic<int> attempted{0}; ///< smooth_before() entered
        std::atomic<int> before_bbox{0}; ///< base smooth_before said no: on the bounding box
        std::atomic<int> before_unrounded{0}; ///< base smooth_before said no: could not round
        std::atomic<int> before_phase_b_not_offset{
            0}; ///< Phase B: on an input surface, neither placed nor relaxed
        std::atomic<int> before_phase_b_enveloped_background{0}; ///< Phase B: envelope-held
        std::atomic<int> before_phase_b_enveloped_offset{0}; ///< Phase B: on-offset AND held
        std::atomic<int> offset_attempted{0}; ///< reached the smoother with the offset term
        std::atomic<int> offset_accepted{0}; ///< ... and the smoother kept the new position
        std::atomic<int> interior_attempted{0}; ///< reached it without one
        std::atomic<int> region_attempted{0}; ///< ... of which sat on another region's boundary
        /// Phi residual over the offset vertices this pass actually touched, before and after,
        /// in units of 1e-9 so an integer atomic can accumulate a sum and a max.
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
     * @brief Why smoothing does not repair a sliver in its one-ring. Same counters as 2D:
     * offered / reached / fixed / stationary. See TopoOffsetTriMesh::m_needle_pre.
     */
    mutable wmtk::threading::enumerable_thread_specific<std::pair<double, Vector3d>> m_needle_pre;
    mutable std::atomic<size_t> m_needle_smooth_offered{0};
    mutable std::atomic<size_t> m_needle_smooth_reached{0};
    mutable std::atomic<size_t> m_needle_smooth_fixed{0};
    mutable std::atomic<size_t> m_needle_smooth_stationary{0};
    mutable std::atomic<size_t> m_needle_smooth_reports{0};

    /// Max AMIPS (the cube root of the stored cell quality) over the tets incident to `vid`.
    /// -1 if it has none.
    double ring_max_quality(size_t vid) const;
    /// AMIPS of one tet, the cube root of cell_quality(); the number every log line reports.
    double tet_amips(const size_t tid) const { return std::cbrt(cell_quality(tid)); }

    /**
     * @brief Scale-invariant flatness: 6 * volume / longest_edge^3.
     *
     * ~0.118 for a regular tet, -> 0 as the four vertices become coplanar, and independent of
     * size. AMIPS saturates at the MAX_ENERGY sentinel while this keeps resolving. The 2D twin
     * is face_flatness().
     */
    double tet_flatness(size_t tid) const;

    /// The full post-mortem on why nothing removes the flat cells; see the 2D twin.
    void needle_forensics() const;

    /// Genesis: flatness transitions recorded at the operation hooks. {op, parent, child}.
    void record_flatness(const char* op, double parent_flat, size_t child_tid) const;
    mutable std::atomic<size_t> m_flat_created_split{0};
    mutable std::atomic<size_t> m_flat_created_collapse{0};
    mutable std::atomic<size_t> m_flat_worsened_split{0};
    mutable std::atomic<size_t> m_flat_genesis_reports{0};
    static constexpr double kFlatThreshold = 1e-3;
    /// The flattest tet in the collapse's ring before it ran, for record_flatness().
    mutable wmtk::threading::enumerable_thread_specific<double> m_collapse_parent_flatness;
    /// The collapse survivor's own sizing scalar, recorded in collapse_edge_before() and put back
    /// in collapse_edge_after() when sizing_collapse_min is false; see that key.
    mutable wmtk::threading::enumerable_thread_specific<double> m_collapse_survivor_sizing;
    void log_smooth_trace() const;

    /// Are the tracked region boundaries actually contained by anything? The 3D twin of
    /// log_region_edge_mask_health(): a class-0 face whose corners' masks AND to zero is held by
    /// nothing. Called at construction and at each turn so the two can be compared.
    void log_region_face_mask_health(const std::string& when) const;

    /// Which tracked faces are outside their envelope, and by how much, per real member tube.
    /// Diagnostic only; the 3D twin of the 2D function of the same name.
    void audit_surface_containment(const std::string& when) const;

    /// How many front placements found the vertex already outside its own envelope on entry.
    /// The invariant is 0. A run total.
    mutable std::atomic<int> m_placement_env_entry_outside{0};
    /// How many front placements had their accepted step projected back into the vertex's
    /// region tubes. A run total.
    mutable std::atomic<int> m_placement_projected{0};
    /// How many front placements were solved tangentially -- along the vertex's own region
    /// boundary rather than along the field normal. A run total.
    mutable std::atomic<int> m_placement_tangential{0};

    ////// wmtk::TetOptimizerMesh hooks

    /// Is this vertex on a region boundary -- a tag boundary, or the domain wall. Derived, not
    /// stored, exactly as in 2D.
    bool vertex_is_on_region(const size_t vid) const
    {
        return m_vertex_extra[vid].m_is_on_region || !m_vertex_attribute[vid].on_bbox_faces.empty();
    }

    /// The three helpers of the per-tag envelope dispatch.
    uint64_t tag_bits(const CellTag& tags) const
    {
        uint64_t bits = 0;
        for (const int64_t t : tags) {
            const auto it = m_tag_bit.find(t);
            if (it != m_tag_bit.end()) bits |= (uint64_t(1) << it->second);
        }
        return bits;
    }

    /// The tag boundaries this vertex lies on -- the raw mask gated on the vertex still being
    /// region geometry at all. The gate keeps the mask honest: the split's endpoint AND
    /// over-claims on chords through the interior, and the front is built by splitting exactly
    /// such edges.
    uint64_t vertex_boundary_mask(const size_t vid) const
    {
        return vertex_is_on_region(vid) ? m_vertex_extra[vid].m_boundary_mask : uint64_t(0);
    }

    /// A face lies on a boundary only if all of it does: the AND of its corners' masks. The 3D
    /// twin of edge_mask(), which ANDs two.
    uint64_t face_mask(const std::array<size_t, 3>& vids) const
    {
        return vertex_boundary_mask(vids[0]) & vertex_boundary_mask(vids[1]) &
               vertex_boundary_mask(vids[2]);
    }

    /// Diagnostic only: which tag boundaries the incident tets say this face lies on right now
    /// -- the same symmetric difference init_surfaces_and_boundaries() classified by. Only
    /// trustworthy while the tet tags are still the input's own.
    uint64_t face_boundary_bits(const Tuple& f) const
    {
        const std::optional<Tuple> opp = f.switch_tetrahedron(*this);
        if (!opp) {
            return tag_bits(m_tet_attribute[f.tid(*this)].tag); // domain wall
        }
        const auto& t0 = m_tet_attribute[f.tid(*this)].tag;
        const auto& t1 = m_tet_attribute[opp->tid(*this)].tag;
        CellTag diff;
        std::set_symmetric_difference(
            t0.begin(),
            t0.end(),
            t1.begin(),
            t1.end(),
            std::inserter(diff, diff.begin()));
        return tag_bits(diff);
    }

    /// The envelope a simplex with this boundary mask is contained in, or null. Zero bits: no
    /// container. One bit: that tag's envelope. Several: a memoized IntersectionEnvelope, which
    /// is containment-only and must never be returned from smoothing_energy_envelope().
    std::shared_ptr<SampleEnvelope> envelope_for_mask(uint64_t mask) const;

    /**
     * @brief Class-0 faces -- every region boundary, the input complex and the domain wall
     * included -- carry a containment requirement; the offset surface does not, except in Phase
     * A where m_offset_envelope holds it where the last smoothing pass left it.
     *
     * The 3D twin of surface_envelope_for_edge(), keyed on the vertices because every caller is
     * an operation asking about a triangle it is about to create. Null means "no containment
     * requirement", which the base handles by skipping the check.
     */
    std::shared_ptr<SampleEnvelope> surface_envelope_for_face(
        const std::array<size_t, 3>& vids) const override
    {
        uint64_t mask = face_mask(vids);
        bool all_offset = true;
        for (const size_t v : vids) {
            all_offset = all_offset && m_vertex_extra[v].m_is_on_offset;
        }
        // The ambiguous case: all corners can be on region boundaries and on the offset surface
        // at once. The corner-mask AND is then necessary but not sufficient for the face lying
        // on a shared boundary; ask the face's own class, the only record that distinguishes a
        // chord from a boundary. Reading the slot is safe here: a split child never reaches
        // this branch, and the m_is_surface_fs guard leaves both constraints standing when a
        // slot is illegible.
        if (mask != 0 && all_offset) {
            if (const auto found = try_tuple_from_face(vids)) {
                const size_t fid = std::get<1>(*found);
                if (m_face_attribute[fid].m_is_surface_fs) {
                    if (face_is_offset(fid)) {
                        mask = 0; // an offset face lies on no region boundary
                    } else {
                        all_offset = false; // a region face is not the offset surface
                    }
                }
            }
        }
        const std::shared_ptr<SampleEnvelope> base = containment_for(mask, all_offset);
        if (base || m_deform_tags.empty()) return base;
        // deform_others' ops-only tube: a released boundary is held by no mask -- its vertices
        // were freed so smoothing can carry the object -- which would leave the operations free
        // to decimate and reposition it. A face the masks and the offset class do not claim, but
        // which lies on a released boundary by its incident tets' current tags, is held to the
        // tube around the boundary's current shape.
        if (const auto found = try_tuple_from_face(vids)) {
            if (face_borders_released_boundary(std::get<0>(*found))) return released_envelope();
        }
        return nullptr;
    }

    /// Surface edges may be flipped, as a topology-preserving diagonal flip. Both tracked
    /// surfaces need it: the offset surface is re-triangulated constantly.
    bool allow_surface_swap() const override { return true; }
    bool check_surface_topology() const override { return m_offset_params.perform_sanity_checks; }

    /**
     * @brief The offset surface is the one tracked surface with no envelope, in either role.
     *
     * The pull must be a real envelope, never a composite; so a junction vertex is pulled toward
     * its most-violated member tube instead, one real envelope per attempt, while the
     * containment intersection below enforces the full constraint. As in 2D.
     */
    std::shared_ptr<SampleEnvelope> smoothing_energy_envelope(const size_t vid) const override
    {
        if (m_vertex_extra[vid].m_is_on_offset && !vertex_is_on_region(vid)) {
            return nullptr;
        }
        const uint64_t mask = vertex_boundary_mask(vid);
        if (mask == 0) {
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

    /// ... and it is not contained by one either, except in Phase A. Both families composed --
    /// not a choice between them; see containment_for().
    std::shared_ptr<SampleEnvelope> smoothing_containment_envelope(const size_t vid) const override
    {
        return containment_for(vertex_boundary_mask(vid), m_vertex_extra[vid].m_is_on_offset);
    }

    /**
     * @brief Phase B placement of a front vertex: the shared smoother with the offset's options,
     * or the 1-D solve along the vertex's move direction under front_normal_projection. See
     * FrontSmooth3d.cpp.
     */
    bool smooth_front_vertex_phase_b(const Tuple& t);
    /// ||grad F|| at front vertex vid along its move direction, F the objective
    /// smooth_front_vertex_phase_b() minimises. +inf if unmeasurable.
    double front_vertex_normal_gradient(size_t vid) const;
    /// The line a front vertex is placed along: the field normal, or that normal projected into
    /// the boundary surface (onto its crease) where an input envelope holds it.
    Vector3d front_vertex_move_direction(size_t vid) const;
    /// Whether the 1-D placement at vid is trapped by the alignment term: a live front face at
    /// or past perpendicular to the field AND the alignment term's 1-D gradient opposing the
    /// placement term's along the move direction, at a vertex stationary off its level set.
    bool front_vertex_alignment_traps_1d_solve(size_t vid) const;
    /// The vertex's convergence measure divided by its bar, per front_conv_criterion: 1 is the
    /// bar. Infinite when unmeasurable.
    double front_vertex_conv_ratio(size_t vid) const;
    /// The edge test divided by its bar (1 = bar): the sagitta of the level set over the chord
    /// (a, b) against front_conv_rel x target_distance; -1 unmeasurable.
    double edge_conv_ratio(size_t a, size_t b) const;
    mutable size_t m_front_gradient_worst_vid =
        static_cast<size_t>(-1); ///< argmax of phase_b_front_gradient_linf()
    /// The field's unit direction at front vertex vid (zero where grad Phi vanishes).
    Vector3d front_vertex_normal(size_t vid) const;
    /// The Phase B objective of front vertex vid with the vertex at x: AMIPS of its one-ring +
    /// phase_b_front_energy(). What the measure above differentiates.
    std::shared_ptr<polysolve::nonlinear::Problem> phase_b_front_objective(
        size_t vid,
        const Vector3d& x) const;
    /// Phase B's offset terms, handed to the shared smoother for a front vertex it is placing
    /// (null in Phase A and for a front vertex an input envelope also pins) -- plus, under
    /// deform_others, the rest-shape AMIPS of the deformable cells in the vertex's ring.
    std::shared_ptr<polysolve::nonlinear::Problem> smoothing_extra_energy(
        const size_t vid) const override
    {
        std::shared_ptr<polysolve::nonlinear::Problem> front;
        if (phase_places_front() && m_offset_potential && m_vertex_extra[vid].m_is_on_offset &&
            vertex_boundary_mask(vid) == 0) {
            front = phase_b_front_energy(vid, potential_ptr_for(vid));
        }
        const std::shared_ptr<polysolve::nonlinear::Problem> rest = rest_energy_for_vertex(vid);
        if (!front) return rest;
        if (!rest) return front;
        auto sum = std::make_shared<optimization::EnergySum>();
        sum->add_energy(front);
        sum->add_energy(rest);
        return sum;
    }

    // ------- deform_others: other input regions deform instead of being envelope-held -------

    /// The released tags. Filled by release_deformable_regions(); empty = feature inactive.
    std::set<int64_t> m_deform_tags;
    /// The source tags (offset_selection's tags_involved), stored at release so the ops-only
    /// tube's face classification applies the same never-freed rule the release did.
    std::set<int64_t> m_source_tags;
    /// The released boundaries' ops-only tube: a SampleEnvelope around the current deformed
    /// boundaries, consulted only by surface_envelope_for_face(). Rebuilt lazily by
    /// released_envelope() when m_released_tube_dirty says a smoothing accept may have moved
    /// the boundary.
    mutable std::shared_ptr<SampleEnvelope> m_released_envelope;
    mutable std::atomic<bool> m_released_tube_dirty{false};
    mutable std::mutex m_released_mutex;
    /// The current released-boundary tube, rebuilt first if dirty. Null when nothing is
    /// released or no released-boundary face exists.
    std::shared_ptr<SampleEnvelope> released_envelope() const;
    /// Whether this face lies on a released region's boundary, by the incident tets' current
    /// tag symmetric difference -- the same test the release freed vertices by.
    bool face_borders_released_boundary(const Tuple& f) const;
    /// A cell deforms when it is background (label 0), tagged, and every tag it carries was
    /// released -- a cell shared with a held region must not deform freely.
    bool cell_is_deformable(size_t tid) const;
    /// Plastic medium: under deform_others every background cell is plastic, its rest shape
    /// re-stamped before every operation group. The band and the complex are not plastic.
    bool m_plastic_active = false; ///< set in optimize_offset() when deform_others
    bool cell_is_plastic(size_t tid) const
    {
        return m_plastic_active && m_tet_attribute[tid].label == 0;
    }
    /// Stamp rest := current for every plastic cell; called before every operation group.
    void stamp_plastic_rests();
    /// The plastic vertex's smoothing: rest-shape AMIPS over its ring, nothing else.
    bool smooth_plastic_vertex(const Tuple& t);
    /// A band cell that is a released object's material: every non-output tag released, at
    /// least one present.
    bool cell_is_released_band(size_t tid) const;
    /// Stamp rest := the cell's current corner positions (oriented order). No-op for
    /// non-deformable cells.
    void stamp_rest_cell(size_t tid);
    /// Drop the released tags' envelopes and stamp every deformable cell's rest. Called once
    /// from optimize_offset() when deform_others is set.
    void release_deformable_regions();
    /// The rest-shape AMIPS over the deformable cells of vid's one-ring, weighted like the
    /// shared smoother weights its AMIPS term; null when the ring has none.
    std::shared_ptr<polysolve::nonlinear::Problem> rest_energy_for_vertex(size_t vid) const;
    /// The two offset terms for a front vertex: the zeroth-order OffsetEnergy3D and the
    /// first-order AlignEnergy3D (one residual per incident live front face). Defined in
    /// FrontSmooth3d.cpp.
    std::shared_ptr<polysolve::nonlinear::Problem> phase_b_front_energy(
        size_t vid,
        const std::shared_ptr<const OffsetPotential3D>& pot) const;

    /**
     * @brief The loop's quality metric: TetWild's own outside Phase B, the max of AMIPS and the
     * Phi residual (each over its own target) in Phase B. See the 2D twin.
     */
    std::tuple<double, double> optimization_quality_stats() override;

    /// stop_energy outside Phase B, 1.0 in it -- in the same units as the line above.
    double optimization_stop_metric() const override
    {
        return m_phase != OptPhase::B ? wmtk::TetOptimizerMesh::optimization_stop_metric() : 1.;
    }

    /// Samples per offset face; see offset_face_samples().
    int offset_residual_samples() const { return m_offset_params.offset_residual_samples; }

    /// The residual scale, derived from the criterion: half the gradient tolerance over the
    /// level-set slope squared, in length units. Same expression as 2D.
    double offset_residual_tolerance() const
    {
        const double s = m_offset_potential ? m_offset_potential->level_set_slope() : 1.;
        return std::max(0.5 * offset_gradient_tolerance() / (s * s), 1e-16);
    }

    /// The gradient_norm_rel bar: front_conv_rel x a measured reference (m_gradient_reference,
    /// never measured on the single-phase path, so this sits at the floor there; the
    /// single-phase bar uses m_front_gradient_reference instead). Same as 2D.
    double offset_gradient_tolerance() const
    {
        return std::max(m_offset_params.front_conv_rel * m_gradient_reference, 1e-16);
    }

    /// The scale offset_gradient_tolerance() is a fraction of; 0 on the single-phase path.
    double gradient_reference() const { return m_gradient_reference; }

    /// Stop the run if any reachable band vertex has left the potential's support. Called once
    /// per turn and once on the band as constructed.
    void check_offset_within_support(const char* when) const;

    /// The band's distance error, split by whether the optimizer can do anything about it.
    /// Same struct as 2D, face samples in place of edge samples.
    struct DistanceSplit
    {
        double max_reachable = 0., avg_reachable = 0.;
        double max_pinned = 0.;
        size_t n_reachable = 0, n_pinned = 0;
        double max_at_vertex = 0., max_in_face = 0.;
        size_t n_outside_support = 0;
        size_t worst_outside_vid = static_cast<size_t>(-1);
        double worst_outside_dist = 0.;
    };
    DistanceSplit distance_deviation_split() const;

    /// The same split over the quantity the loop converges on: the Phi residual, as a length.
    DistanceSplit residual_split() const;
    /// Which vertices lie on the band's outer surface. Shared by every measurement.
    std::vector<bool> band_vertex_mask() const;

    /// The furthest any offset-surface vertex sits from the input complex, by BVH. 0 when no
    /// offset exists yet. Sizes dhat in init_offset_potential().
    double max_band_vertex_distance() const;
    /// |dist(vid, input complex) - target_distance|. Diagnostic: the Euclidean offset.
    double band_vertex_distance_error(const size_t vid) const;

    /// How far vid is from the level set Phi = c, as a length.
    double band_vertex_residual(const size_t vid) const;

    /// A quantity sampled at points INSIDE an offset-surface face.
    struct FaceSamples
    {
        double max = 0.;
        double sum = 0.;
        size_t n = 0;
    };

    /**
     * @brief The interior lattice a face is sampled on, handed to `visit` one point at a time:
     * every (i, j, l) with i + j + l = k + 2 and each >= 1, so k = 1 is the centroid and the
     * counts are 1, 3, 6, 10 for k = 1..4. The 2D twin is for_each_offset_edge_sample().
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

    /// The Phi residual at `offset_residual_samples` interior points of offset face `f`. Returns
    /// nothing for a face with an unreachable corner. The 2D twin is offset_edge_samples().
    FaceSamples offset_face_samples(const Tuple& f) const;

    /**
     * @brief The convergence criterion's own split: ||grad (Phi - c)^2|| at band vertices plus
     * the face-interior chord diagnostic and the normal-aligned reference quantity. Same fields
     * as the 2D GradientSplit, face samples in place of edge samples.
     */
    struct GradientSplit
    {
        double max_reachable = 0., avg_reachable = 0.;
        double max_pinned = 0.;
        size_t n_reachable = 0, n_pinned = 0;
        double max_at_vertex = 0., max_in_face = 0.;
        double max_in_face_pinned = 0.;
        double max_normal_aligned = 0.;
        size_t n_face_samples = 0;
        size_t n_skipped_inverted = 0, n_skipped_unrounded = 0;
        size_t worst_vid = static_cast<size_t>(-1);
    };
    /// @param include_face_samples false skips the face-interior half (the expensive one).
    GradientSplit gradient_split(bool include_face_samples = true) const;

    /**
     * @brief The "energy_gradient" criterion and the single phase's states: placed / travelling
     * / pressed / stuck per front vertex, and the refinable chords. The same struct as 2D.
     */
    struct EnergyCriterion
    {
        double max_vertex = 0., max_edge = 0.; ///< ratios to the bar (1 = bar)
        double bar = 1.;
        size_t n_vertices = 0, n_edges = 0, n_unmeasurable = 0;
        size_t n_pressed = 0, n_edges_pressed = 0;
        size_t worst_vid = static_cast<size_t>(-1);
        Vector3d worst_edge_mid = Vector3d::Zero();
        double worst_edge_len = 0.;
        size_t n_edges_over = 0, n_edges_over_on_level = 0;
        double max_edge_on_level = 0.;
        Vector3d worst_on_level_mid = Vector3d::Zero();
        double tube = 0.;
        size_t n_placed = 0, n_travelling = 0, n_pressed_on = 0, n_stuck = 0;
        size_t n_pressed_touching = 0;
        size_t n_at_floor = 0;
        size_t worst_stuck_vid = static_cast<size_t>(-1);
        double worst_stuck_rho = 0.;
        struct Refinable
        {
            size_t a, b;
            double sag, len;
        };
        std::vector<Refinable> refinable;
        bool vertices_ok() const { return max_vertex <= bar; }
        bool edges_ok() const { return max_edge <= bar; }
        bool converged() const { return vertices_ok() && n_unmeasurable == 0; }
        bool converged_single() const { return converged() && refinable.empty(); }
        double ratio() const { return bar > 0. ? std::max(max_vertex, max_edge) / bar : 0.; }
    };
    EnergyCriterion energy_criterion();
    /// Whether a front vertex touches another front, the input or a region boundary through a
    /// background tet -- the topological fact behind the `pressed` state.
    bool front_vertex_touches_other(size_t vid) const;
    /// The edge length that would bring a front chord's sag under the tube: 3/4 L
    /// (tube / sag)^(1/p) capped at L/2, with the exponent p measured from how the level set
    /// turns across the chord. Same formula as 2D.
    double front_chord_target(size_t va, size_t vb, double len, double sag, double tube) const;

    /// The resolution rule: sets the target length at each refinable edge's ends from
    /// front_chord_target(), graded outward. Returns the vertices changed.
    size_t refine_front_from_sag(const std::vector<EnergyCriterion::Refinable>& edges);
    /// The energy criterion as measured when the loop converged; the final Phase A runs after
    /// it and the verdict must not be re-measured on that mesh.
    std::optional<EnergyCriterion> m_energy_verdict;
    /// The interpolation residual of front edge (a, b), see EnergyCriterion. -1 unmeasurable.
    double edge_interpolation_residual(size_t a, size_t b) const;

    /// The normal at an offset vertex: the unit vector from the nearest point on the input
    /// complex to the vertex. Zero where undefined. Same definition as 2D.
    Vector3d offset_vertex_normal(const size_t vid) const;

    /// Turn a residual_split()'s outside-support tally into the hard error.
    void report_outside_support(const char* when, const DistanceSplit& s) const;
    /// Whether vid is a band vertex the optimizer could still place at target_distance. An
    /// envelope-held offset vertex is pinned, as is one on the domain boundary. Same rule as 2D.
    bool band_vertex_is_reachable(const size_t vid) const
    {
        if (m_vertex_extra[vid].m_is_on_offset && vertex_boundary_mask(vid) != 0) return false;
        return !vertex_is_on_domain_boundary(vid);
    }

    /// Set by the placement when a vertex's last visit stopped on a constraint; the criterion
    /// counts such a vertex as placed. Same as 2D.
    std::vector<char> m_placement_pressed;

    /// TetWild's stall-driven sizing refinement, verbatim; Phase A only. See the 2D twin.
    size_t refine_sizing_around_worst(double max_metric) override;

    /// Why Phase A is stuck: a census of the tets stuck-refine is about to chase. The 3D twin of
    /// log_stuck_refine_census().
    void log_stuck_refine_census(double max_metric, double filter_energy);

    /// For every element above `filter_energy`, why its edges cannot be split: short / valence /
    /// contain / free. The 3D twin of log_refine_block_census().
    void log_refine_block_census(const std::string& when, double filter_energy) const;

    /// Instrumentation only: which operation manufactures the MAX_ENERGY needles.
    bool collapse_quality_allowed(size_t v1, double q, double ring_max) const override;

    mutable std::atomic<size_t> m_deg_split_created{0};
    mutable std::atomic<size_t> m_deg_collapse_offered{0};
    mutable std::atomic<size_t> m_deg_collapse_allowed{0};
    mutable std::atomic<size_t> m_deg_collapse_by_ringmax{0};
    mutable std::atomic<size_t> m_deg_collapse_by_stop{0};
    mutable std::atomic<size_t> m_deg_collapse_by_unrounded{0};
    std::array<size_t, 6> m_deg_prev_counts{{0, 0, 0, 0, 0, 0}};

    /// Where the first needles come from -- a tripwire, capped at kNeedleReports.
    void report_needle(const char* op, size_t tid, double parent_q) const;
    static constexpr size_t kNeedleReports = 12;
    /// What counts as a needle for the tripwire, in AMIPS -- deliberately far below MAX_ENERGY.
    static constexpr double kNeedleQuality = 1e6;
    mutable std::atomic<size_t> m_needle_reports{0};

    /// Population scan at a named moment. Reports the count and the worst few.
    void needle_scan(const char* when) const;

    /// Quantised centroids of the MAX_ENERGY tets at the previous stuck-refine, for the overlap
    /// line. Diagnostic only.
    std::set<std::tuple<long, long, long>> m_stuck_prev_cells;
    size_t m_stuck_calls = 0;

    /// TetWild's bare collapse passes are off for the offset, as TriWild's are in 2D: with no
    /// length gate the quality test alone demolishes the band, and the sizing field cannot
    /// refuse a collapse.
    bool optimization_bare_coarsen_passes() const override { return false; }

    /// Max of the two normalized criteria (AMIPS over stop, residual over tolerance) on this
    /// face; >= 1 means it fails at least one. The coarsen-mode collapse accept reads it.
    double face_criterion_rel(const Tuple& f) const;
    /// AMIPS of a cell over stop_energy -- the 3D twin of TriOptimizerMesh::quality_rel().
    double cell_quality_rel(const size_t tid) const;
    /// ... and the worst of the (up to two) cells a face separates.
    double amips_rel_at_face(const Tuple& f) const;

    /**
     * @brief Put the frames beside the run's own output, and rename them into one timeline:
     * <output>_NNNNN.vtu with one "NNNNN<tab>r<round><phase><pass>_<op>" line per frame in
     * <output>_frames.txt. Exactly the 2D scheme; see TopoOffsetTriMesh.
     */
    void write_optimization_debug_output(const std::string& path) override
    {
        const char ph = (m_phase == OptPhase::A) ? 'A' : (m_phase == OptPhase::B ? 'B' : 'S');
        if (m_ab_round != m_debug_last_round || ph != m_debug_last_phase) {
            m_debug_last_round = m_ab_round;
            m_debug_last_phase = ph;
            m_debug_pass = 0;
        }
        std::string label = path;
        if (path.rfind("debug_", 0) == 0) {
            label = fmt::format(
                "r{}{}{}{}",
                m_ab_round,
                ph,
                ++m_debug_pass,
                m_debug_pass_name.empty() ? std::string() : "_" + m_debug_pass_name);
        } else if (path.rfind("phase_", 0) == 0) {
            label = fmt::format("r{}{}_end", m_ab_round, ph);
        }
        const size_t idx = m_debug_seq++;
        append_frame_label(idx, label);
        write_vtu(m_offset_params.output_path + fmt::format("_{:05d}", idx));
    }
    /// The pass the next debug frame belongs to; set by the single-phase loop before each
    /// operation group, and by the smoothing sweeps. The 2D base carries this itself.
    std::string m_debug_pass_name;

    /// One line of <output>_frames.txt; truncates the file on the first frame.
    void append_frame_label(size_t idx, const std::string& label) const;

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
        const std::vector<std::string>& tag_names,
        const std::string& sheet_name = "");

    /// check that the ambient tag does not overlap with any other tags
    bool ambient_assert();

    /// label input simplicial complex simplices, as defined in m_offset_params.offset_selection
    void label_input_complex();

    /// check if the input complex is empty. Only valid after calling init_from_image(...).
    bool empty_input_complex();

    /**
     * @brief Build the input complex's BVH and keep the extraction the potential needs, and
     * number the complex's connected pieces. Must be called after init_from_image(...) and
     * label_input_complex().
     */
    void init_input_complex_bvh();

    /// Build the smooth offset potential from the extraction init_input_complex_bvh() kept.
    void init_offset_potential();

    /// The input complex as Phi's primitives: the BOUNDARY triangles of the label-1 tet region
    /// plus the complex's isolated triangles, every edge of those triangles plus the complex's
    /// wires, and its isolated points. Filled by init_input_complex_bvh().
    MatrixXd m_phi_V;
    MatrixXi m_phi_E;
    MatrixXi m_phi_F;
    std::vector<int> m_phi_P;

    /// label connected simplicial complex components (simplices labelled 1 or 2)
    size_t flood_fill();

    std::vector<std::array<size_t, 3>> get_faces_by_condition(
        std::function<bool(const FaceAttributes&)> cond) const;

    //// overriden splits/invariants
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
    bool split_tet_before(const Tuple& t) override;
    bool split_tet_after(const Tuple& t) override;
    bool invariants(const std::vector<Tuple>& tets) override;
    //// overriden splits/invariants

    /**
     * @brief TetWild over the input mesh, before any of the offset exists, held only by the
     * per-tag region envelopes. The 3D twin of TopoOffsetTriMesh::pre_optimize_input_mesh().
     */
    void pre_optimize_input_mesh();

    /// main function from which all others are called
    void execute_offset(const std::filesystem::path& output_file);

    /// Simplistic marching tets: all edges with one vertex labelled 0 and the other 1/2 are
    /// split at the midpoint. No target_distance enters construction.
    void marching_tets();

    //// simplicial embedding stuff
    bool is_simplicially_embedded() const;
    bool tet_is_simp_emb(const Tuple& t) const;
    void simplicial_embedding();
    //// simplicial embedding stuff

    /// update 'tags' data for tets in the offset region (tets labelled 2)
    void set_offset_tet_tags();

    /// verify that the closed offset region (simplices labelled 1 or 2) form a manifold region.
    bool offset_is_manifold();

    //// output stuff
    /// Sample the potential on the plane through the box centre normal to its shortest extent
    /// and write it as <path>_phi.vtu, n x n samples. See phi_grid_resolution; 0 disables.
    void write_phi_grid(const std::string& path, int n) const;

    void write_input_complex(const std::string& path);
    void write_vtu(const std::string& path);
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
        std::map<simplex::Edge, EdgeAttributes> existing_e;
        std::map<simplex::Face, FaceSnapshot> existing_f;
        int splitf_label;
        std::map<size_t, TetAttributes> tets;
    };
    wmtk::threading::enumerable_thread_specific<FaceSplitCache> face_split_cache;

    struct TetSplitCache
    {
        std::array<size_t, 4> v_ids;
        std::map<simplex::Edge, EdgeAttributes> existing_e;
        std::map<simplex::Face, FaceSnapshot> existing_f;
        TetAttributes tet;
    };
    wmtk::threading::enumerable_thread_specific<TetSplitCache> tet_split_cache;

    bool swap_capture_tag(const std::vector<size_t>& tids);
    /// The tag swap_after_cells writes onto the tets the swap created, chosen in `before`.
    wmtk::threading::enumerable_thread_specific<CellTag> m_swap_tag;
    /// The construction label shared by every cell of the swap's ring, captured alongside.
    wmtk::threading::enumerable_thread_specific<int> m_swap_label;

public:
    // substructure functions

    bool is_order_2_edge(const Tuple& e) const;
    bool is_order_2_edge(const std::array<size_t, 2>& e) const;

    bool vertex_is_on_surface(const size_t vid) const override;

    bool face_is_on_surface(const size_t fid) const override;

    size_t get_order_of_vertex(const size_t vid) const override;
    /// Compute the vertex order for every vertex.
    void init_vertex_order();

private: // helpers
    /**
     * @brief determine if any tag from tag1 is also present in tag2.
     * @note if tag2 is empty (ambient), return true if tag1 is empty, otherwise false
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

    /// sort edge simplices in place by decreasing edge length
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
    /// assign each vertex a partition id (by spatial Morton order). A no-op if NUM_THREADS == 0.
    void compute_vertex_partition();

    size_t get_partition_id(const Tuple& loc) const
    {
        return m_vertex_attribute[loc.vid(*this)].partition_id;
    }

    /// all one-ring vertices through input simplices (labelled 1 or 2)
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

    /// reset connected component assignments.
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
