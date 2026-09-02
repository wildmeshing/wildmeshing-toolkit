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
#include <wmtk/optimization/EnergySum.hpp>
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
 * wmtk::TriOptimizerMesh::VertexAttributes. The three flags here say which tracked surface a
 * vertex belongs to; the base's m_is_on_surface is their union, and they are not exclusive -- a
 * vertex where the offset boundary meets another region's boundary carries both.
 */
class VertexExtra2d
{
public:
    int label = 0;
    bool m_is_on_input = false; // on the input complex
    bool m_is_on_offset = false; // on the offset boundary itself
    bool m_is_on_region = false; // on some OTHER tag region's boundary
    /// Where this vertex stood at the start of the turn (single phase), so the convergence states
    /// can read its net movement -- a vertex jiggling back and forth in a pressed seam has a large
    /// summed motion and a small net one, and only the net one says whether the seam has settled.
    /// A split copies it on, so a new vertex reads as moved for the turn it was born in.
    Vector2d m_turn_start = Vector2d::Zero();
    bool m_turn_start_valid = false;

    /**
     * @brief Which tag boundaries this vertex lies on -- one bit per input tag, ambient included.
     * See TopoOffsetTriMesh::m_tag_envelopes for what the bits dispatch to.
     *
     * Seeded in init_surfaces_and_boundaries() from the input partition, then propagated by the
     * operations: a split's new vertex takes the AND of its endpoints (it lies on a boundary only
     * if the whole edge did), a collapse's survivor the OR (it carries both vertices' geometry).
     */
    uint64_t m_boundary_mask = 0;

    /// Churn instrumentation: which split pass created this vertex, from
    /// wmtk::TriOptimizerMesh::m_op_epoch; 0 means not created by an optimization split. Read only
    /// by collapse_after_vertex(). Assigned at each split, never OR'd -- a recycled slot carries a
    /// dead vertex's epoch.
    uint32_t m_born_epoch = 0;
};


/// Per-edge construction label; the surface tags themselves are the base's
/// wmtk::SurfaceTagAttributes. Registered with m_edge_attr_group.
class EdgeExtra2d
{
public:
    int label = 0;
    /// The edge lies on the input's curve group (within its envelope): selectable as the complex
    /// by name, held in that group's tube. Re-derived by classify_curve_edges(); nothing
    /// propagates it through split or collapse, so it is read only before the optimization starts.
    bool on_curve = false;
};


/// Per-face construction label. The region tag lives in the base's FaceAttributes::tags.
/// Registered with m_face_attr_group.
class FaceExtra2d
{
public:
    int label = 0;
    /**
     * Rest shape (deform_others): the face's corners when it last changed topologically, in the
     * oriented order consolidate_mesh() preserves. Stamped for every deformable face at release
     * and re-stamped by the operation after-hooks for every face an accepted split / collapse /
     * swap changed, never by smoothing -- a child left on its parent's rest reads det F ~ 1/2 and
     * fights to regrow.
     */
    bool rest_valid = false;
    std::array<Eigen::Vector2d, 3> rest_pos;
};


/**
 * @brief The offset's 2D mesh, on the shared 2D optimizer.
 *
 * Mirrors TopoOffsetTetMesh: the construction phase is entirely its own, and the optimization
 * phase that follows is wmtk::TriOptimizerMesh's.
 *
 * Two surfaces are tracked. Every tag-region boundary (input complex and domain wall included)
 * keeps the primary class 0 and is held in its tags' envelopes, as triwild holds its input; the
 * offset boundary is OFFSET_SURFACE_CLASS, in 2D exactly the edges across which the incident face
 * labels differ, so label_offset_boundary() derives it rather than storing it. Class-0 edges may
 * move within their tubes; only the offset one is driven toward target_distance.
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
    /// Tag id of the input's curve group (the .msh line elements), or -1. An open curve has no
    /// face set whose boundary it is, so it is selectable only through this tag: offset_selection
    /// naming it makes the curve the complex and the band grows on both of its sides.
    int64_t m_curve_tag = -1;
    /// The curve group as loaded, kept because the classification below is redone on demand.
    MatrixXd m_curve_V;
    MatrixXi m_curve_E;
    /**
     * @brief Mark the mesh edges that lie on the input's curve group (EdgeExtra2d::on_curve).
     *
     * Geometric, against the curve's own tube (the same eps the tag envelopes use), because
     * triwild writes its curves with their own vertices and there is no index to match on.
     *
     * Called whenever the complex is labelled, not once at load: the flag is a property of an edge
     * and nothing propagates it through split and collapse, so an operation pass shreds it.
     * Re-deriving it is exact and costs one tube query per edge.
     */
    void classify_curve_edges();
    /**
     * @brief The input complex as loaded. Built once, never rebuilt.
     *
     * It answers the Euclidean distance to the input, a diagnostic rather than the definition of
     * the offset -- see m_offset_potential, which is what the optimization is driven by.
     * init_input_complex_bvh() has one call site, before execute_offset() runs, so this holds the
     * original geometry however the elements representing the complex are later remeshed.
     *
     * That invariant is load-bearing: rebuilding from the live mesh would redefine the offset
     * distance in terms of a surface the optimizer had just moved, and the convergence criterion
     * would be measuring the mesh against itself.
     *
     * The only structure over the input complex. For offset_field "euclidean" the potential shares
     * this very object as its query engine, which is why it is a shared_ptr. Containment is not
     * its job -- the per-tag region envelopes (m_tag_envelopes) hold the complex in place.
     */
    std::shared_ptr<SimplicialComplexBVH> m_input_complex_bvh;

    /**
     * @brief The smooth offset potential, and with it the definition of the offset itself.
     *
     * The offset boundary is the level set Phi = c. Built from the same extraction as
     * m_input_complex_bvh, in the same call, so the two describe the same geometry and the same
     * never-rebuilt rule applies. See OffsetPotential for what Phi is.
     *
     * shared_ptr because OffsetEnergy2D holds one per smoothing call.
     */
    std::shared_ptr<OffsetPotential2D> m_offset_potential;

    /**
     * @brief One field per connected piece of the input complex, and which one each band vertex
     * is placed on.
     *
     * m_offset_potential above is built over the whole selected complex: the sum of every piece's
     * barrier for the smooth potential, the distance to the nearest piece for the Euclidean field.
     * Neither is the field a front should be placed on where two pieces are close -- the sum has
     * no level set at all across a narrow gap, so both fronts are pushed through the background
     * strip until inversion.
     *
     * A region is a connected piece, not a tag: one tag covering two pieces that never touch would
     * make them share a field and bring that bridging back. Pieces are the connected components of
     * the captured complex under vertex connectivity (two pieces meeting at a point share an
     * offset there, so they share a field), computed once in init_input_complex_bvh() so the
     * numbering is fixed for the whole run. simplicial_embedding() is what makes this correspond
     * to the band: no background triangle can touch two disjoint pieces, so the band's connected
     * components are the disjoint offsets.
     *
     * A band grown from one piece is placed on that piece's field alone: Phi_A = c for band A,
     * Phi_B = c for band B. Where the two would overlap, each front is pulled outward by its own
     * field and held by the strip's quality bar -- a symmetric local minimum with a thin gap of
     * background between the fronts, which is the topological offset.
     *
     * The map from band to region is assign_band_regions(): a flood fill over the band faces,
     * seeded from every band face with a complex vertex, whose piece is read off the captured
     * complex geometrically. A face reachable from two regions and a vertex on faces of two
     * regions read -2 and fall back to the union field. m_offset_potential is kept for everything
     * that is not per-vertex: the support (dhat), the viewer's grid, the report.
     */
    int m_n_regions = 0; ///< connected pieces of the input complex; one field each
    std::vector<std::shared_ptr<OffsetPotential2D>> m_region_potentials; ///< one per piece
    std::vector<int64_t> m_phi_vert_region; ///< per m_phi_V row: region index
    std::vector<int64_t> m_phi_seg_region; ///< per m_phi_E row: region index, -1 unknown
    std::vector<int64_t> m_phi_face_region; ///< per m_phi_F row: region index, -1 unknown
    std::vector<int64_t> m_phi_point_region; ///< per m_phi_P entry: region index, -1 unknown
    std::vector<int> m_face_region; ///< per face: band's region, -1 none, -2 reached from two
    std::vector<int> m_vertex_region; ///< per vertex: region of its band faces, -1 / -2 as above
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
    const OffsetPotential2D& potential_for_region(const int region) const
    {
        return (region >= 0 && size_t(region) < m_region_potentials.size())
                   ? *m_region_potentials[size_t(region)]
                   : *m_offset_potential;
    }
    const OffsetPotential2D& potential_for(const size_t vid) const
    {
        return potential_for_region(vertex_region(vid));
    }
    /// The same selection as potential_for(), as the pointer the energies take a share of. Null
    /// only when m_offset_potential is, which the front placement paths test for.
    std::shared_ptr<const OffsetPotential2D> potential_ptr_for(const size_t vid) const
    {
        const int r = vertex_region(vid);
        return (r >= 0 && size_t(r) < m_region_potentials.size()) ? m_region_potentials[size_t(r)]
                                                                  : m_offset_potential;
    }
    const OffsetPotential2D& potential_for_edge(const size_t va, const size_t vb) const
    {
        return potential_for_region(edge_region(va, vb));
    }
    const OffsetPotential2D& potential_for_face(const size_t fid) const
    {
        return potential_for_region(fid < m_face_region.size() ? m_face_region[fid] : -1);
    }

    /**
     * @brief One containment envelope per input tag, ambient included. Both phases.
     *
     * E_t is a tube of half-width m_envelope_eps around region t's boundary segments as the input
     * mesh carried them, built in init_surfaces_and_boundaries() before offset construction: the
     * band's tags replace a face's own, so an envelope built later would be a tube around a curve
     * truncated at the band. A simplex on several boundaries is held by the intersection of its
     * tags' tubes (envelope_for_mask()), which pins junction points to the junction itself.
     *
     * m_envelope (the base's pointer) survives as a UnionEnvelope over these members, purely so
     * the shared engine's direct uses of it -- the collapse_edge_before point check and the
     * "segment does not exist yet" fallback here -- keep union semantics.
     *
     * Interior edges of a region are not held by these: identical tag sets on both sides land in
     * no bucket, so a filled complex's interior is free to optimise.
     */
    std::map<int64_t, std::shared_ptr<SampleEnvelope>> m_tag_envelopes;

    /**
     * @brief The per-tag boundary polyline, with the adjacency an arclength walk needs.
     *
     * The same segments m_tag_envelopes[tag] was built from, in the same indexing, so
     * SampleEnvelope::nearest_point_feature()'s `feature_id` (a polyline vertex index when
     * on_corner, else a segment index) indexes straight into these. Built once beside the
     * envelopes, from the input partition, and never rebuilt -- the envelope is a tube around this
     * curve, so the two must not be able to drift apart.
     *
     * The component keeps its own copy because SampleEnvelope's arrays are protected and this mesh
     * does not subclass it. `at_vertex` answers what the envelope cannot: which segments meet at a
     * polyline vertex. Where three or more meet the walk stops, the continuation being ambiguous.
     */
    struct TagPolyline2d
    {
        std::vector<Eigen::Vector2i> E; ///< segments, indexing m_env_polyline_V
        std::vector<std::vector<int>> at_vertex; ///< polyline vertex -> incident segment ids
    };
    /// Shared vertex array for every TagPolyline2d: the positions as init_surfaces_and_boundaries()
    /// saw them, indexed by the mesh vid at construction. Never renumbered -- the envelopes hold
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
     * A simplex can be on both, so this is always their intersection, never an either/or.
     *
     * Separate from m_isect_cache because the members differ in lifetime: the tag envelopes live
     * for the whole run, m_offset_envelope is rebuilt after every Phase B.
     * rebuild_offset_envelope() clears this and must keep doing so -- a stale entry holds the
     * previous round's offset tube and would pin the boundary to where it was two rounds ago.
     * Guarded by m_isect_mutex.
     */
    mutable std::map<uint64_t, std::shared_ptr<SampleEnvelope>> m_offset_isect_cache;

    /**
     * @brief The containment a simplex with this region mask, on/off the offset front, must
     * satisfy -- the intersection of everything that holds it, or null if nothing does.
     *
     * The single place the two containment families are composed. `region_mask` dispatches through
     * envelope_for_mask() (itself an intersection when the mask is multi-bit, which is what pins a
     * junction to the junction); `on_offset` adds m_offset_envelope, but only in Phase A -- Phase
     * B is the pass whose job is to move the offset boundary, so there the result is the region
     * tubes alone.
     */
    std::shared_ptr<SampleEnvelope> containment_for(uint64_t region_mask, bool on_offset) const;

    /**
     * @brief Move `x` back inside every region tube this vertex lies on. True if it ended up
     * inside all of them.
     *
     * The projection half of the projected-gradient placement: an offset vertex a region envelope
     * also holds takes the same unconstrained step on the same objective as any other, and this
     * restores its validity. Refusal cannot express "go as far as you may" -- a vertex whose every
     * trial step leaves the tube never moves at all -- while projecting keeps the component of the
     * step the tube allows, which for a vertex on a boundary curve is motion along that curve.
     *
     * Never asks a composite: nearest_point() and squared_distance() are non-virtual on
     * SampleEnvelope and would bind to an IntersectionEnvelope's base subobject, whose BVH was
     * never built (TagEnvelopes.hpp). So this walks the mask's real members from m_tag_envelopes
     * and composes them by alternating projection onto the worst-violated one; each
     * nearest_point() lands x on that member's curve, so the rounds converge a junction onto the
     * intersection of its curves.
     *
     * Returns false if the alternation did not converge, which is the caller's signal to keep
     * the entry position rather than commit an invalid one.
     */
    bool project_into_containment(size_t vid, Vector2d& x) const;

    /**
     * @brief Which tag's boundary curve a vertex slides along, or -1.
     *
     * The bit of its mask whose curve passes closest to it. For the common multi-bit case that
     * choice is immaterial: a mask carries a bit per tag on either side of the boundary, so an
     * interface between two regions gives both bits and both curves contain it. Where the curves
     * genuinely differ the line search still tests containment against every member tube, so
     * picking the nearest chooses the parameterization, never the constraint.
     */
    int64_t tangent_curve_tag(size_t vid, const Vector2d& x) const;

    /**
     * @brief March `s` of arclength along tag `tag`'s boundary polyline from `x`'s foot on it.
     *
     * The reduced coordinate of the tangential placement: the constraint is eliminated rather
     * than enforced, so every point this returns is on the curve and needs no containment test of
     * its own. Returns false where the walk cannot continue -- an open end, or a polyline vertex
     * where three or more segments meet and the continuation is ambiguous -- with `out` holding
     * the furthest point reached, the end of the feasible interval.
     *
     * Corners are not special-cased, deliberately. The walk crosses any corner; what stops a
     * vertex sliding past a sharp one is the incident chord leaving its tube, which the caller's
     * backtracking finds. That reproduces the true bound (roughly eps/sin(theta) for a turn of
     * theta, unbounded along a straight run) with no angle threshold anywhere.
     */
    bool walk_along_curve(int64_t tag, const Vector2d& x, double s, Vector2d& out) const;

    /**
     * @brief The unit tangent of tag `tag`'s curve at `x`'s foot, or false if there is none.
     *
     * From nearest_point_feature()'s seg_normal, rotated a quarter turn. At a polyline vertex the
     * tangent is two-valued, so this takes the incident segment best aligned with the descent the
     * caller is about to attempt -- the standard reading of a one-sided derivative at a kink.
     */
    bool curve_tangent(int64_t tag, const Vector2d& x, const Vector2d& prefer, Vector2d& tau) const;

    /**
     * @brief Which half of the alternating optimization is running.
     *
     * The two criteria are optimized in turn, not jointly. The 2D counterpart of
     * TopoOffsetTetMesh::OptPhase.
     *
     * Phase A is TriWild and nothing else: same operations, gates, sizing field and stall-driven
     * refinement, with no offset energy term, acceptance criterion or stop metric. Its one
     * addition is m_offset_envelope.
     *
     * Phase B moves the offset boundary and nothing else: smoothing passes against the offset
     * energy, run to a fixed point, with no envelope on the offset (it is what has to travel)
     * and no topological operations at all.
     *
     * The sizing field is shared and both phases write it: Phase A through TriWild's stall
     * refinement on element quality, Phase B through the Phi residual of the faces smoothing
     * could not place.
     */
    /// Single is the mode the 2D run uses: TriWild's own loop with the front placed by Phase B's
    /// objective inside the smoothing passes. It follows B wherever the smoother is concerned
    /// (which objective a front vertex gets, no offset tube while it moves) and A everywhere the
    /// loop is concerned (quality stats and the stop metric are TriWild's).
    enum class OptPhase { A, B, Single };

    /// Whether the smoother places front vertices against the offset objective: Phase B, and
    /// the single-phase mode that does the same thing inside TriWild's passes.
    bool phase_places_front() const { return m_phase != OptPhase::A; }

    /// Which phase is running. Read by every hook that differs between them; see OptPhase.
    OptPhase m_phase = OptPhase::A;

    // Phase B is one sweep over every vertex per pass: a front vertex is placed by
    // smooth_offset_vertex_backtracking(), an interior one relaxed by the shared smoother, in
    // mesh order -- see smooth_before().

    /// The final Phase A: front vertices are not smoothed (see smooth_before()).
    bool m_freeze_front = false;

    // No Phi test gates collapse and swap: the envelope is the constraint, and the offset
    // criterion belongs to Phase B's own placement. The coarsening bar is applied where 3D
    // applies it -- absolute, and only in m_coarsen_mode.

    /**
     * @brief The tube the offset boundary may not leave during Phase A, of half-width
     * offset_envelope_rel x target_distance. Rebuilt at the end of every Phase B from the
     * boundary as that phase left it, which is what lets the boundary travel across rounds.
     * Non-null once the offset exists; whether it constrains is containment_for()'s phase test,
     * not the pointer. Unlike m_tag_envelopes, which must never be rebuilt.
     */
    std::shared_ptr<SampleEnvelope> m_offset_envelope;

    /// Rebuild m_offset_envelope from the current offset-boundary segments, and drop the
    /// intersections memoized against the old one. Called when the offset is created and at the
    /// end of every Phase B.
    void rebuild_offset_envelope();

    /// Hard error if any vertex is on both the input complex and the offset boundary -- a state
    /// no placement satisfies. Called at construction and after every phase.
    void check_no_vertex_on_both_surfaces(const char* when) const;

    /// TriWild's loop, the front placed inside its smoothing passes.
    void optimize_offset_single_phase();

    /// Max over the front vertices Phase B places of ||grad F||, F the vertex's full Phase B
    /// objective (AMIPS + the offset terms, as the shared smoother assembles it). The pass stop.
    double phase_b_front_gradient_linf();
    /// Its value on the band as constructed, measured once before round 1: the reference the
    /// Phase B pass stop is a fraction of.
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

    /**
     * @brief SurfaceTagAttributes::m_surface_class: which of the two tracked surfaces an edge
     * belongs to. Same scheme as 3D.
     *
     * OFFSET is the surface the optimization places at target_distance. Everything else -- the
     * input complex, another body's outline, an overlap seam, the domain wall -- keeps the primary
     * class 0 and is envelope-checked by the shared operations exactly as in triwild and simwild.
     * The distinction has to exist: filing a region boundary under OFFSET drives placement at
     * vertices nowhere near target_distance and leaves the sizing field refining there forever.
     * Class 0 is not split further -- the boundary mask says which tubes hold a simplex, per tag.
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

        // As in 3D. The per-vertex Newton solver logs a line per smoothing attempt at info level,
        // which is one line per vertex per pass and buries the run's own output.
        optimization::deactivate_opt_logger();
    }

    ~TopoOffsetTriMesh() override = default;

    /**
     * @brief Place a vertex, keeping its exact and rounded coordinates in step.
     *
     * As in 3D: the offset works in doubles, so every vertex it places is rounded, but m_pos must
     * still be filled because the shared split's exact-midpoint fallback reads it.
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
    /// ... and whether it bounds a region -- any tracked edge that is not the offset boundary.
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
     * The offset boundary has no stored definition in 2D -- it is exactly the edges across which
     * the incident face labels differ, so it falls out of the labelling and is recomputed here
     * once. An edge with one incident face is on the domain boundary and is tagged bbox instead,
     * which is what stops the box from collapsing.
     */
    void label_offset_boundary();

    /**
     * @brief Whether face `fid` belongs to the closed offset region, read from its label.
     *
     * The region is the offset band (label 2) plus the input complex it wraps (label 1), both set
     * at construction from geometry rather than tags, and every operation carries the label onto
     * the faces it creates, so this is exact. Tags cannot express the distinction: nothing stops
     * the band's output tag already appearing elsewhere in the input mesh, and such a face would
     * read as offset band, so the offset energy would drag an unrelated region to the level set.
     */
    bool face_in_region(const size_t fid) const;

    /// Whether face `fid` is part of the input complex (as opposed to the offset band).
    /// Label 1, assigned by label_input_complex() from the user's selection expression.
    bool face_is_input_complex(const size_t fid) const;

    /**
     * @brief The substructure the link condition is evaluated against, derived not cached.
     *
     * substructure_link_condition() is only as good as these answers. Cached edge tags are
     * refreshed once per iteration, which is too coarse: the split pass creates edges the tagging
     * never classified, so the collapse pass that follows would evaluate against a substructure
     * that no longer describes the mesh -- which is why split and collapse tear the region
     * together while each is safe alone. Computing from the face labels on demand cannot go stale.
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
     * The absolute error |dist(v, input complex) - target_distance| over the offset-boundary
     * vertices only. The max is what the optimization converges against -- the offset is only as
     * good as its worst-placed vertex, and an average hides a stretch far off the target.
     * Mirrors TopoOffsetTetMesh::compute_distance_deviation().
     */
    std::pair<double, double> compute_distance_deviation() const;

    /// The vertex compute_distance_deviation() last found the max at, and a dump of everything
    /// that could be stopping it from moving. Diagnostic only.
    mutable size_t m_worst_dist_vid = static_cast<size_t>(-1);
    void log_worst_dist_vertex() const;

    /// Whether this edge is on the band's outer surface, recomputed from the tags on every call
    /// -- the live counterpart of edge_is_offset(), for use inside the operation passes.
    bool edge_is_offset_surface_live(const Tuple& e) const;

    /// Seed the sizing field from the offset's current edge lengths (paper Sec. 5.3.3, Step 1),
    /// once, before the first operation pass. Without it the field starts at the background
    /// target length and the first collapse pass decimates the offset.
    void init_offset_sizing_field();

    /// {max_dist_err, avg_dist_err, max_phi_residual, avg_phi_residual, max_grad, avg_grad,
    /// max_grad_at_vertex, max_grad_in_edge}. max_grad is the convergence criterion -- the full
    /// placement-gradient norm at band vertices -- so max_grad_at_vertex repeats it and
    /// max_grad_in_edge is the chord diagnostic; the rest are diagnostics. One entry for the whole
    /// run, as in 3D.
    std::vector<std::array<double, 8>> optimization_metrics;
    /// {split-born vertices, recollapsed, recollapsed in the immediately following collapse
    /// pass} per A/B round, in step with op_counts. See VertexExtra2d::m_born_epoch.
    std::vector<std::array<int, 3>> churn_counts;
    /// {splits, collapses, swaps} per A/B round -- one entry per round the driver runs, including
    /// the round that converges, as deltas rather than running totals. Phase B does no topological
    /// work, so a round's entry is exactly what its Phase A did. This does not mirror
    /// optimization_metrics, which is a single whole-run summary.
    std::vector<std::array<int, 3>> op_counts;
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
    /// Operations refused because they would have left an offset-boundary face over tolerance.
    std::atomic<int> iter_cnt_collapse_offset_reject{0};
    std::atomic<int> iter_cnt_swap_offset_reject{0};
    /// Splits of an offset-boundary edge: offered, accepted.
    std::atomic<int> iter_cnt_split_offset_before{0};
    std::atomic<int> iter_cnt_split_offset{0};
    /// Parent face labels for an optimization split, keyed by the apex vertex opposite the split
    /// edge -- shared by both children of the same parent, so it names them afterwards. Keyed and
    /// consumed exactly as TriOptimizerMesh::split_edge_after does its own FaceAttributes cache,
    /// so the label lands wherever the tags do; the endpoints are what resolve each child's apex.
    struct OptSplitCache2d
    {
        std::map<size_t, int> face_label;
        size_t v1_id = 0;
        size_t v2_id = 0;
        /// The endpoints' mask AND, captured before the split (3D's rule at both of its split
        /// sites). Consumed by split_after_vertex() behind the parent edge's own class gate, which
        /// keeps a chord's midpoint maskless so the AND cannot over-claim through one. Never
        /// derived from the incident faces' current tags: the band retag empties the live
        /// symmetric difference on every region edge it swallows.
        uint64_t edge_bits = 0;
        /// Diagnostic: the two parent faces' AMIPS before the split, so split_after_vertex() can
        /// say whether a needle child came from a healthy parent or an already unscoreable one.
        double parent_q_max = -1.;
        /// Same question in the scale-invariant measure, which keeps resolving after AMIPS has
        /// saturated at MAX_ENERGY. Min over the parents: the flattest thing the split inherited.
        double parent_flatness = 1.;
    };
    wmtk::threading::enumerable_thread_specific<OptSplitCache2d> m_opt_split_cache;

    bool marching_split_edge_before(const Tuple& t);
    bool marching_split_edge_after(const Tuple& t);

    /**
     * @brief Reject any collapse that violates the substructure link condition.
     *
     * The base applies it only when both endpoints already sit on a tracked surface or the bbox,
     * which is the right rule for tetwild and simwild but not here: the offset region is a thin
     * band, and a collapse with only one endpoint on the boundary can still pinch its two sides
     * together and make the region non-manifold. The offset asks unconditionally.
     */
    bool collapse_edge_before(const Tuple& t) override;

    /**
     * @brief Reject a flip whose new edge already exists.
     *
     * Flipping (a,b) to (c,d) when c and d are already joined creates a second edge between the
     * same pair. Across a thin offset band that is how the two sides get stitched together and the
     * region stops being manifold. The base refuses tracked-surface edges but not this.
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
     * Bookkeeping, not positioning, and it lives here because of when the base calls the hooks:
     * the containment check inside split_edge_after() runs on both new segments and so reaches
     * surface_envelope_for_edge() and the endpoints' boundary masks. split_after_vertex() runs
     * after that check; this hook is the last one the base offers before it.
     *
     * Getting it wrong is silent in the dangerous direction: children still holding whatever
     * occupied their recycled fid slots classify as "not a region boundary", which yields a null
     * envelope, and a null envelope makes surface_segment_is_outside() return false -- containment
     * skipped rather than failed, so the offset polyline decays unchecked.
     *
     * Position is left entirely to the base; this always returns its result unchanged.
     */
    bool split_adjust_position(size_t v_new, const std::vector<Tuple>& children) override;

    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    /**
     * @brief Identification only -- no operation refuses the domain wall through these.
     *
     * The wall is a tracked region boundary like every other one: init_surfaces_and_boundaries()
     * tags its edges m_is_surface_fs, masks its vertices with ambient's bit and puts its segments
     * in ambient's envelope, so refinement, coarsening, flips and smoothing are governed by the
     * same containment, merge rules and link conditions that govern the input complex. As in 3D,
     * the hooks carry no categorical wall refusal of their own.
     *
     * What still reads these two:
     *  - band_vertex_is_reachable(): a wall-clipped offset vertex is booked pinned for the
     *    convergence criterion, since gating on it would deadlock the run.
     *  - the base's own wall rules, which stand apart from the component: the collapse
     *    on_bbox_faces subset rule and the smoothing wall freeze -- which the component's
     *    smooth_before() deliberately bypasses in favour of envelope containment.
     *  - diagnostics.
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
     * @brief Classify every region boundary, build the per-tag containment envelopes, and tag the
     * domain wall -- once, from the input mesh, before offset construction runs.
     *
     * The 2D twin of TopoOffsetTetMesh::init_surfaces_and_boundaries(), called from the same place
     * for the same reason: the band's tags replace a face's own rather than joining them, so an
     * envelope built afterwards would be a tube around a curve truncated at the band.
     *
     * A region boundary is an edge whose two incident faces carry different tag sets; it enters
     * the bucket of every tag on exactly one side (the symmetric difference). An edge with only
     * one incident face is the domain wall and enters its single face's tags' buckets, which is
     * how ambient's envelope comes to hold the box. Requires the face tags to be set, which
     * init_from_image() does just above the call.
     */
    void init_surfaces_and_boundaries();

    /// Set VertexExtra2d::m_is_on_input from the construction labels, once label_input_complex()
    /// has evaluated the selection. Separate from init_surfaces_and_boundaries(), which runs
    /// earlier and can only see tag boundaries. The 3D twin is mark_input_complex_vertices().
    void mark_input_complex_vertices();

    /**
     * @brief Warn if the offset band has grown into the domain boundary.
     *
     * When target_distance exceeds the clearance between the input complex and the bounding box,
     * construction runs out of room and the band's outer boundary becomes the box itself.
     *
     * Two things then go wrong invisibly: those vertices are on the bbox and cannot be moved, so
     * the target distance is unreachable there, and compute_distance_deviation() cannot even see
     * them -- it skips edges with no opposite face, which is what a band edge on the domain
     * boundary is, so the clipped stretch enters neither max_dist_err nor avg_dist_err. The run
     * then looks like a near-miss and is a structural failure, hence a warning, not a debug line.
     */
    void warn_if_offset_reaches_domain_boundary() const;

    /**
     * @brief What smoothing did with each class of vertex, per pass.
     *
     * The base's SmoothRejectCounters says why a move was refused; it cannot say what kind of
     * vertex was asking. Every vertex takes the same path -- the shared smoother -- so what is
     * worth counting here is the dispatch: how many attempts were on the offset boundary (the
     * ones carrying the offset term), how many on another region's boundary, and how many were
     * turned away before the smoother saw them at all.
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
        ///
        /// The one number that says whether the offset term is doing anything: an acceptance count
        /// cannot, because the quality veto refuses a move outright rather than shortening it, so
        /// a pass can accept most of what it tries and still move nothing badly placed.
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
     * Interleaved smoothing is on by default, so every needle-adjacent vertex is visited after
     * every topological pass. These counters say what happens when it is:
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
     * independent of size. AMIPS saturates at the MAX_ENERGY sentinel while this keeps resolving,
     * which is what the genesis tracking needs: "this face got flatter" is a statement AMIPS
     * cannot make once it is unscoreable.
     */
    double face_flatness(size_t fid) const;

    /**
     * @brief The full post-mortem on why nothing removes the flat faces.
     *
     * For the worst faces by flatness, reports per edge every gate that decides whether an
     * operation may touch it: length against the collapse gate (4/5 l s-bar) and the split gate
     * (4/3 l s-bar), whether it is force-split queued, is_edge_on_surface (swap_weight returns
     * lowest() for a surface edge, so it is never swapped) and swap_weight itself. Plus a scan for
     * coincident vertices, with whether each pair shares an edge -- a pair that does not is
     * geometry no local operation can reach.
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
    /// The collapse survivor's own sizing scalar, recorded in collapse_edge_before() and put back
    /// in collapse_edge_after() when sizing_collapse_min is false; see that key.
    mutable wmtk::threading::enumerable_thread_specific<double> m_collapse_survivor_sizing;
    void log_smooth_trace() const;


    /**
     * @brief Are the tracked region boundaries actually contained by anything?
     *
     * A class-0 edge is dispatched to an envelope by its boundary mask, the symmetric difference
     * of its two faces' tags. An edge whose faces carry the same tags has an empty difference, so
     * envelope_for_mask() gives it nullptr: tracked as a region boundary and held by nothing, in
     * either phase. Construction cannot produce one, so a non-zero count here is a hole opened
     * afterwards. Called at construction and at each Phase B entry so the two can be compared.
     */
    void log_region_edge_mask_health(const std::string& when) const;

    /**
     * @brief Which tracked edges are outside their envelope, and by how much.
     *
     * The shared pass driver's sanity_checks() reports "Edge [a, b] is outside!" but not which
     * envelope refused it, and the answer forks the diagnosis: offset-class (mask 0) means Phase A
     * moved the offset boundary out of the tube holding it where Phase B left it; region-class
     * (mask != 0) means a tag-region boundary has drifted off the input partition, refused by that
     * tag's tube or by the intersection of several at a junction.
     *
     * Reports per-endpoint distance to each real member tube. A multi-bit mask dispatches an
     * IntersectionEnvelope, which must never be asked squared_distance (TagEnvelopes.hpp: its BVH
     * is null), so the members are walked individually instead of querying the composite.
     *
     * Call it at construction as well as inside the loop: an edge already outside before any
     * operation runs is a construction defect, a different bug. Diagnostic only.
     */
    void audit_surface_containment(const std::string& when) const;

    /// How many Phase B offset placements found the vertex already outside its own envelope on
    /// entry. The post-step projection pulls it back in, but the invariant is 0: a nonzero count
    /// means construction or Phase A leaves offset vertices outside their region tube. A run total.
    mutable std::atomic<int> m_placement_env_entry_outside{0};

    /// How many Phase B offset placements had their accepted step projected back into the
    /// vertex's region tubes -- expected wherever the offset coincides with a region boundary, and
    /// not a problem: it counts constrained motion along a boundary curve. Read it against the
    /// EnvelopeBlocked count, where a projection that could not be committed lands. A run total.
    mutable std::atomic<int> m_placement_projected{0};

    /// How many Phase B offset placements were solved tangentially -- reduced to arclength along
    /// the vertex's own tag boundary curve rather than stepped freely in 2D, expected for every
    /// offset vertex a region envelope also holds. Read it against ChordBlocked, which counts the
    /// visits whose slide an incident chord leaving its tube cut off. A run total.
    mutable std::atomic<int> m_placement_tangential{0};

    ////// wmtk::TriOptimizerMesh hooks

    /**
     * @brief Is this vertex on a region boundary -- a tag boundary, or the domain wall.
     *
     * Derived, not stored, exactly as in 3D. Both halves are already maintained: m_is_on_region by
     * the split/collapse hooks, on_bbox_faces by set_intersection of the split endpoints and by
     * the collapse rule that a wall vertex may only merge into one at least as constrained.
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
     * @brief The tag boundaries this vertex lies on -- the raw mask gated on the vertex still
     * being region geometry at all.
     *
     * The gate is not redundant, it is what keeps the mask honest. m_boundary_mask propagates by a
     * bare AND of a split's endpoints, which over-claims: an edge whose two ends happen to share a
     * bit hands that bit to its midpoint even when the edge is a chord through the interior, and
     * the offset front is built by splitting precisely such edges. So the mask says which
     * boundaries and vertex_is_on_region() says whether the vertex is on one at all.
     */
    uint64_t vertex_boundary_mask(const size_t vid) const
    {
        return vertex_is_on_region(vid) ? m_vertex_extra[vid].m_boundary_mask : uint64_t(0);
    }

    /// A segment lies on a boundary only if both ends do: the AND of its endpoints' masks. The
    /// 2D twin of face_mask(), which ANDs three.
    uint64_t edge_mask(const std::array<size_t, 2>& vids) const
    {
        return vertex_boundary_mask(vids[0]) & vertex_boundary_mask(vids[1]);
    }

    /**
     * @brief Diagnostic only: which tag boundaries the incident faces say this edge lies on right
     * now -- the same symmetric difference init_surfaces_and_boundaries() classified by.
     *
     * Nothing dispatches or propagates from this. It is only trustworthy while the face tags are
     * still the input's own: execute_offset() replaces the tags of every face the band grows
     * through, after which this is empty across every region edge the band swallowed, so deriving
     * split masks from it mints uncontained region vertices (log_region_edge_mask_health counts
     * the divergence). New vertices take the endpoints' mask AND behind the parent edge's class
     * gate instead -- 3D's rule at both of its split sites -- which is what stops a chord, not
     * being a region-class edge, from over-claiming a tube a full target_distance away.
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
     * memoized IntersectionEnvelope over the members -- inside means inside every tube, which pins
     * junction geometry to the junction. Containment-only for the multi-bit case: the composite
     * implements just the virtual is_outside queries, so it must never be returned from
     * smoothing_energy_envelope(), whose pull calls the non-virtual nearest_point.
     */
    std::shared_ptr<SampleEnvelope> envelope_for_mask(uint64_t mask) const;

    /**
     * @brief Class-0 segments -- every region boundary, the input complex and the domain wall
     * included -- carry a containment requirement; the offset boundary does not.
     *
     * The envelope holds the other tag regions where they are, and the input complex too. That
     * half is exactly TriWild's input envelope: the complex may be split, collapsed and smoothed,
     * and this is what bounds how far the result may drift from the geometry as loaded.
     *
     * The offset boundary is exempt in Phase B, where it is the surface the optimization exists to
     * move and a tube around its initial position would cap how far it can travel. In Phase A it
     * is held by m_offset_envelope instead -- a tube of one Phi tolerance around wherever Phase B
     * last left it, rebuilt each round -- which turns "do not degrade the offset" from a
     * per-operation criterion into a geometric constraint every shared operation already honours.
     *
     * Null means "no containment requirement", which the base handles by skipping the check.
     */
    std::shared_ptr<SampleEnvelope> surface_envelope_for_edge(
        const std::array<size_t, 2>& vids) const override
    {
        // Boundary geometry first, in both phases: a segment on any tag-region boundary may not
        // drift out of that boundary's tube, and one on several boundaries -- a junction -- is
        // held in their intersection. The mask carries the input complex too, since every complex
        // simplex lies on tag boundaries, and E_t is built from the input mesh before construction
        // touches it, so the per-tag tubes hold the as-loaded geometry.
        //
        // Keyed on the vertices, not the edge: every caller is an operation asking about a segment
        // it is about to create or has just created, whose own edge attributes are not written
        // yet. The endpoints' masks are, maintained by the operations themselves (AND at a split,
        // OR at a collapse).
        uint64_t mask = edge_mask(vids);
        bool all_offset = true;
        for (const size_t v : vids) {
            all_offset = all_offset && m_vertex_extra[v].m_is_on_offset;
        }

        // The ambiguous case: both endpoints can be on region boundaries and on the offset front
        // at once -- a real state, and the point of tracking the two families separately. The
        // endpoint-mask AND is then necessary but not sufficient for the segment lying on a shared
        // boundary: two vertices on different junctions can share a tag bit by coincidence and be
        // joined by an offset chord nowhere near that tag's curve, which a container it was never
        // meant to satisfy then refuses to refine. Ask the edge's own class, the only record that
        // distinguishes a chord from a boundary.
        //
        // Reading the slot is safe here and would not be unconditionally: the base checks a
        // split's two child segments before writing their attributes, so their slots still hold a
        // recycled edge's class. A split child never reaches this branch -- its mask and its
        // offset flag are mutually exclusive by construction, so one of `mask` and `all_offset` is
        // always empty -- and the m_is_surface_fs guard leaves both constraints standing rather
        // than dropping one when a slot is illegible.
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
        // offset where Phase B left it; Phase B is what moves it, so it contributes nothing there
        // -- and null before the offset exists at all, which is the pre-pass.
        const std::shared_ptr<SampleEnvelope> base = containment_for(mask, all_offset);
        if (base || m_deform_tags.empty()) return base;
        // deform_others' ops-only tube: a released boundary is held by no mask -- its vertices
        // were freed so smoothing can carry the object -- which would leave the operations free
        // to decimate and reposition it. A segment the masks and the offset class do not claim,
        // but which lies on a released boundary by its incident faces' current tags, is held to
        // the tube around the boundary's current shape. A fresh split child can misread its
        // recycled face slots for this one check, and is at worst skipped or over-held once.
        if (const auto found = try_tuple_from_edge(vids)) {
            if (edge_borders_released_boundary(std::get<0>(*found))) return released_envelope();
        }
        return nullptr;
    }

    /**
     * @brief No per-vertex positional constraint. The per-tag envelopes close that hole
     * structurally -- the same deletion 3D made to its lower-strata point refusal.
     *
     * An isolated point of the complex only ever arises where two or more selected tags meet (the
     * boolean selection can only label an isolated simplex whose face star is tag-heterogeneous),
     * so the edges radiating from it are tag boundaries and its boundary mask carries several
     * bits. smoothing_containment_envelope() therefore hands the smoother an IntersectionEnvelope
     * -- within eps of every curve it lies on -- which pins it to the junction, and the pull
     * toward the most-violated member drags it back if it strays. The base's hook is pure virtual,
     * so this stays as the honest constant rather than being deleted outright.
     */
    bool smoothing_position_is_allowed(const size_t, const Vector2d&) const override
    {
        return true;
    }

    /**
     * @brief The offset boundary is the one tracked surface with no envelope, in either role.
     *
     * It is the surface the optimization exists to move: a tube around wherever construction left
     * it would cap how far it can ever travel toward the level set. What holds it is the offset
     * term in the objective, not a container.
     *
     * The pull must be a real envelope, never a composite: this hook's consumers call the
     * non-virtual SampleEnvelope queries -- nearest_point and the ExactDistanceEnergy2D trio --
     * which on a composite would bind to the base's null BVH. So a junction vertex (several mask
     * bits) is pulled toward its most-violated member tube instead, one real envelope per
     * attempt, while the containment intersection below enforces the full constraint.
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
     * @brief ... and it is not contained by one either, except in Phase A.
     *
     * Phase A is TriWild and its smoothing minimises AMIPS alone; without a container nothing
     * would stop it relocating the offset boundary for the sake of element shape, which Phase B
     * would then have to undo. Phase B keeps null -- that is the pass whose job is to move it.
     *
     * Boundary geometry is contained in the intersection of its tags' tubes, in both phases: the
     * caller only asks is_outside(segment), which composites answer, so unlike the pull this side
     * may hand out an IntersectionEnvelope.
     */
    std::shared_ptr<SampleEnvelope> smoothing_containment_envelope(const size_t vid) const override
    {
        // Both families, composed -- not a choice between them. A vertex where the offset front
        // meets one or more region boundaries is on all of them at once, and the three cases fall
        // out of one expression: pure-offset (mask 0) gives the offset tube in Phase A and null in
        // Phase B, pure-region gives its tubes' intersection in both, and a junction of the two
        // gives the intersection of everything.
        return containment_for(vertex_boundary_mask(vid), m_vertex_extra[vid].m_is_on_offset);
    }

    /**
     * @brief Phase B placement of a front vertex: the shared smoother with the offset's options.
     *
     * The same 2-D Newton solve, line search and accept tests as any TriWild vertex; the objective
     * carries the offset terms through smoothing_extra_energy(). No quality veto, because a front
     * vertex has to be able to worsen its ring on its way to the level set -- element shape is
     * Phase A's job.
     */
    bool smooth_front_vertex_phase_b(const Tuple& t);
    /// ||grad F|| at front vertex vid, F the objective smooth_front_vertex_phase_b() minimises.
    /// +inf if unmeasurable. The pass stop and the loop's vertex test.
    double front_vertex_normal_gradient(size_t vid) const;
    /// The line a front vertex is placed along: the field normal, or the boundary tangent where
    /// an input envelope holds it. See the definition.
    Vector2d front_vertex_move_direction(size_t vid) const;
    /// Whether the 1-D placement at vid is trapped by the alignment term: a live front edge at
    /// or past perpendicular to the field AND the alignment term's 1-D gradient opposing the
    /// placement term's along the move direction. See the definition.
    bool front_vertex_alignment_traps_1d_solve(size_t vid) const;
    /// The vertex's convergence measure divided by its bar, per front_conv_criterion: 1 is the
    /// bar. See the spec entry for the three measures. Infinite when unmeasurable.
    double front_vertex_conv_ratio(size_t vid) const;
    /// The edge test divided by its bar (1 = bar), per front_conv_criterion; -1 unmeasurable.
    double edge_conv_ratio(const Tuple& e) const;
    mutable size_t m_front_gradient_worst_vid =
        static_cast<size_t>(-1); ///< argmax of phase_b_front_gradient_linf()
    /// The field's outward unit direction at front vertex vid (zero where grad Phi vanishes).
    Vector2d front_vertex_normal(size_t vid) const;
    /// The Phase B objective of front vertex vid with the vertex at x: AMIPS of its one-ring +
    /// phase_b_front_energy(). What the measure above differentiates.
    std::shared_ptr<polysolve::nonlinear::Problem> phase_b_front_objective(
        size_t vid,
        const Vector2d& x) const;
    /// Phase B's offset terms, handed to the shared smoother for a front vertex it is placing
    /// (null in Phase A and for a front vertex an input envelope also pins) -- plus, under
    /// deform_others, the rest-shape AMIPS of the deformable faces in the vertex's ring, in
    /// every phase. Whichever apply are summed; null when neither does.
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
    /// tube's edge classification applies the same never-freed rule the release did.
    std::set<int64_t> m_source_tags;
    /// The released boundaries' ops-only tube: a SampleEnvelope around the current deformed
    /// boundaries, consulted only by surface_envelope_for_edge() -- the dispatch every operation
    /// containment check goes through and no smoothing path does -- so operations preserve the
    /// current shape through remeshing while smoothing stays free to carry the object. Rebuilt
    /// lazily by released_envelope() when m_released_tube_dirty says a smoothing accept may have
    /// moved the boundary.
    mutable std::shared_ptr<SampleEnvelope> m_released_envelope;
    mutable std::atomic<bool> m_released_tube_dirty{false};
    mutable std::mutex m_released_mutex;
    /// The current released-boundary tube, rebuilt first if dirty. Null when nothing is
    /// released or no released-boundary segment exists.
    std::shared_ptr<SampleEnvelope> released_envelope() const;
    /// Whether this edge lies on a released region's boundary, by the incident faces' current
    /// tag symmetric difference -- the same test the release freed vertices by.
    bool edge_borders_released_boundary(const Tuple& e) const;
    /// A face deforms when it is background (label 0), tagged, and every tag it carries was
    /// released -- a face shared with a held region must not deform freely.
    bool face_is_deformable(size_t fid) const;
    /// Plastic medium: under deform_others every background face -- ambient and the other objects
    /// alike -- is plastic, its rest shape re-stamped before every operation group, so smoothing
    /// resists only the increment since the group started and the medium flows instead of behaving
    /// as an elastic solid glued to the walls. The band (label 2) and the complex (label 1) are
    /// not plastic; element quality in the medium is the operation passes' job.
    bool m_plastic_active = false; ///< set in optimize_offset() when deform_others
    bool face_is_plastic(size_t fid) const
    {
        // Every background face, objects included: one material for the medium and the objects.
        // Do not re-add the exclusion for released objects; measured no better -- see git history.
        return m_plastic_active && m_face_extra[fid].label == 0;
    }
    /// Stamp rest := current for every plastic face; called before every operation group.
    void stamp_plastic_rests();
    /// The plastic vertex's smoothing: rest-shape AMIPS over its ring, nothing else -- no
    /// equilateral term, no quality veto, exact inversion as the only accept test.
    bool smooth_plastic_vertex(const Tuple& t);
    /// A band cell that is a released object's material: every non-output tag released, at
    /// least one present. Read by the front placement objective and the rest stamping only --
    /// see the definition for why the band's interior smoothing is left equilateral.
    bool face_is_released_band(size_t fid) const;
    /// Stamp rest := the face's current corner positions (oriented order). No-op for
    /// non-deformable faces.
    void stamp_rest_face(size_t fid);
    /// Drop the released tags' envelopes and stamp every deformable face's rest. Called once
    /// from optimize_offset() when deform_others is set; see the FaceExtra2d::rest_valid doc
    /// for the tracking contract.
    void release_deformable_regions();
    /// The rest-shape AMIPS over the deformable faces of vid's one-ring, weighted like the
    /// shared smoother weights its AMIPS term; null when the ring has none.
    std::shared_ptr<polysolve::nonlinear::Problem> rest_energy_for_vertex(size_t vid) const;
    /// The two offset terms for a front vertex, see smooth_front_vertex_phase_b(): the
    /// zeroth-order OffsetEnergy2D and the first-order AlignEnergy2D (one residual per incident
    /// live front edge). Defined in Optimize2d.cpp, next to the criterion measuring the same
    /// quantities.
    std::shared_ptr<polysolve::nonlinear::Problem> phase_b_front_energy(
        size_t vid,
        const std::shared_ptr<const OffsetPotential2D>& pot) const;

    /**
     * @brief The loop's convergence metric, normalized so that 1.0 means "done".
     *
     * The max of the two criteria this optimization has to meet, each divided by its own target,
     * so mesh_improvement() stops exactly when both are met:
     *
     *   - max face AMIPS over stop_energy -- TriWild's, via quality_rel()
     *   - max Phi residual over (front_conv_rel / 2) * target_distance, over the reachable band
     *
     * The average returned alongside it is the same expression over the two averages, so both
     * numbers live on the same 1.0 scale. Nothing reads the average; it is logged.
     */
    std::tuple<double, double> optimization_quality_stats() override;

    /**
     * @brief 1.0 in Phase B, where the metric is normalized; the base's stop_energy in Phase A.
     *
     * The units are part of "identical to TriWild", and getting this wrong is silent. The pair
     * (optimization_quality_stats, optimization_stop_metric) has to be in one set of units,
     * because refine_sizing_around_worst() derives its filter from the first and then compares
     * that filter against a per-face score. Phase A ranks by m_face_attribute[].m_quality, which
     * is absolute AMIPS, so its metric and its bar must be absolute too -- otherwise
     * select_worst_cells returns nothing, no sizing is refined, and the stall detector fires
     * every iteration and does nothing.
     */
    double optimization_stop_metric() const override
    {
        // Single is TriWild's loop, so its stop metric is TriWild's.
        return m_phase != OptPhase::B ? wmtk::TriOptimizerMesh::optimization_stop_metric() : 1.;
    }

    /// Samples per band edge; see offset_edge_samples(). 0 falls back to a vertex-only
    /// criterion, which is measurably blind to a band too coarse to be the offset.
    int offset_residual_samples() const { return m_offset_params.offset_residual_samples; }

    /// The residual scale, derived from the criterion rather than configured beside it.
    ///
    /// grad E = 2 (Phi - c) grad Phi, so on a field with unit slope at the level set the gradient
    /// bound |grad E| <= g is exactly |Phi - c| <= g/2: half the gradient tolerance, in length
    /// units. It feeds the Phase A offset envelope (offset_envelope_rel x this) and the derived
    /// min_edge_length floor, so loosening the criterion loosens the tube with it.
    double offset_residual_tolerance() const
    {
        // Derived from the tolerance, not from the knob. grad E . n = 2 (Phi - c) (grad Phi . n),
        // so with |grad Phi| = s at the level set a bound g on the normal gradient is the length
        // bound |Phi - c| <= g / (2 s^2). The tolerance is rel x a measured maximum, so this must
        // follow it rather than reading the knob.
        const double s = m_offset_potential ? m_offset_potential->level_set_slope() : 1.;
        return std::max(0.5 * offset_gradient_tolerance() / (s * s), 1e-16);
    }

    /**
     * @brief The convergence tolerance: the bound on |grad (Phi - c)^2| at a band vertex.
     *
     * A fraction of target_distance, which is the right unit: grad E = 2 (Phi - c) grad Phi, and
     * grad Phi is dimensionless for a field whose value is a length, so grad E is a length.
     *
     * The gradient needs no conversion of Phi's value into a length -- it is the stationarity
     * condition of the objective Phase B minimises, so it is the same test for the exact Euclidean
     * field and for the smooth potential alike, which is what lets a reentrant input be judged by
     * the same number as a convex one. See TopoOffsetTetMesh::offset_gradient_tolerance() for the
     * full derivation of the slope normalization.
     */
    double offset_gradient_tolerance() const
    {
        // A fraction of a measured maximum, not of an analytic estimate. m_gradient_reference is
        // max |2 (Phi - c) grad Phi . n| over the offset-surface vertices as constructed, taken
        // once before the A/B loop starts; everything this bar serves -- the convergence criterion
        // and every Phase B local stop -- compares the full gradient norm at those same vertices
        // against it, so both sides come from one measurement pass and no per-field calibration.
        //
        // Never measured on the single-phase path, so this sits at the 1e-16 floor there; the
        // single-phase convergence bar uses m_front_gradient_reference instead.
        return std::max(m_offset_params.front_conv_rel * m_gradient_reference, 1e-16);
    }

    /// max |2 (Phi - c) grad Phi . n| over the initial offset-surface vertices; the scale
    /// offset_gradient_tolerance() is a fraction of. Always 0 on the single-phase path
    /// (never measured); kept for the report.
    double gradient_reference() const { return m_gradient_reference; }

    /**
     * @brief Stop the run if any reachable band vertex has left the potential's support.
     *
     * Beyond dhat, Phi is identically zero with a zero gradient: the vertex is given no direction
     * back, its residual saturates instead of growing, and the sizing field refines around a
     * vertex nothing can move. There is no recovery from that state and no honest report of it
     * either, so it is a hard error -- the answer to it firing is a larger offset_dhat_factor.
     *
     * Called once per optimization iteration, and once on the band as constructed.
     */
    void check_offset_within_support(const char* when) const;

    /**
     * @brief The band's distance error, split by whether the optimizer can do anything about it.
     *
     * Reachable: a band vertex free to be placed at target_distance. Pinned: one that cannot be,
     * whatever the optimizer does -- it lies on the input complex, where the distance is 0 by
     * definition and the envelope keeps it, or on the domain boundary, where construction ran out
     * of room. Neither is an optimization failure, so only the reachable half drives the loop.
     *
     * Both are still reported. compute_distance_deviation() deliberately measures the whole band,
     * so a run whose band is half missing cannot report a small error and "converge" having
     * measured nothing; a pinned vertex out of band is warned about as a construction defect.
     */
    struct DistanceSplit
    {
        double max_reachable = 0., avg_reachable = 0.;
        double max_pinned = 0.;
        size_t n_reachable = 0, n_pinned = 0;
        /// max_reachable, split by where it was measured. The two answer different questions: the
        /// vertex max says the boundary is in the wrong place, the edge max says it is too coarse
        /// to be in the right place -- smoothing versus refinement, so a run that is not
        /// converging needs to know which it is.
        double max_at_vertex = 0., max_in_edge = 0.;
        /// Reachable band vertices that have left the potential's support entirely, and the
        /// worst of them. Collected here rather than in a second traversal because
        /// check_offset_within_support() asks the same question of the same vertices.
        size_t n_outside_support = 0;
        size_t worst_outside_vid = static_cast<size_t>(-1);
        double worst_outside_dist = 0.;
    };
    DistanceSplit distance_deviation_split() const;

    /// The same split over the quantity the loop converges on: the Phi residual, as a length.
    /// Reported beside the Euclidean one so the two offsets can always be compared.
    DistanceSplit residual_split() const;
    /// Which vertices lie on the band's outer surface -- the one that is supposed to sit at
    /// target_distance. Shared by every measurement so they all agree on what "the band" is.
    std::vector<bool> band_vertex_mask() const;

    /// The furthest any offset-boundary vertex sits from the input complex, by BVH. 0 when no
    /// offset exists yet. Sizes dhat in init_offset_potential(); the 3D twin has the same name.
    double max_band_vertex_distance() const;
    /// |dist(vid, input complex) - target_distance|. Diagnostic: the Euclidean offset, which
    /// the level set only coincides with away from reentrant features.
    double band_vertex_distance_error(const size_t vid) const;

    /// How far vid is from the level set Phi = c, as a length. This is what the loop converges
    /// on and what the sizing field refines by.
    double band_vertex_residual(const size_t vid) const;

    /// The residual sampled at points along a band edge -- see offset_edge_samples().
    struct EdgeSamples
    {
        double max = 0.;
        double sum = 0.;
        size_t n = 0;
    };

    /**
     * @brief The Phi residual at `offset_residual_samples` interior points of band edge `e`.
     *
     * The criterion cannot be a vertex criterion: a boundary can have every vertex exactly on the
     * level set while zig-zagging or cutting corners between them, which reads as converged and is
     * not the offset. That is the gap the paper's normal-deviation criterion (Sec. 5.3.3) covered.
     *
     * Sampling the edges is what the potential makes possible and a distance field did not: Phi is
     * defined everywhere, so the offset can be measured anywhere along the band rather than only
     * where the mesh happens to have put a vertex. The same samples feed face_criterion_rel(), so
     * the sizing field refines a band too coarse to represent the offset instead of letting it
     * decimate.
     *
     * Samples are uniform interior points, i/(k+1) for i = 1..k, so k = 1 is the midpoint. Returns
     * nothing for an edge with an unreachable endpoint: a segment running onto the input complex
     * is legitimately closer than target_distance along its length.
     */
    EdgeSamples offset_edge_samples(const Tuple& e) const;

    /**
     * @brief Visit the same interior sample points offset_edge_samples() measures on.
     *
     * Factored out so the residual and the convergence gradient are measured on one lattice -- a
     * criterion sampled on a different set of points from the quantity the sizing field refines by
     * is two measurements pretending to be one. The 3D twin is for_each_offset_face_sample().
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
     * Pinned vertices are reported, not gated, which is where this parts company with
     * residual_split(): a residual is a statement about the boundary, so a pinned vertex off the
     * level set is a real error in the offset the run returns, while a gradient is a statement
     * about the iteration -- folding in a vertex the optimizer never moves would make convergence
     * unreachable by construction.
     *
     * The deciding measure is the full gradient norm at vertices: max_reachable (== max_at_vertex)
     * is max ||2 (Phi - c) grad Phi|| over reachable band vertices, the exact quantity every Phase
     * B local solve stops on, so the run's verdict and the visits' stops are one test. max_in_edge
     * is the edge-interior half and gates alongside the vertex half. max_normal_aligned is
     * |grad E . n| at vertices over reachable and pinned.
     */
    struct GradientSplit
    {
        double max_reachable = 0., avg_reachable = 0.;
        double max_pinned = 0.;
        size_t n_reachable = 0, n_pinned = 0;
        /// max_at_vertex is the vertex half of the criterion, max_in_edge the edge-interior half,
        /// and both gate -- same quantity, same bar. Both are the full norm ||2 (Phi - c) grad
        /// Phi||, and max_in_edge counts only edges whose endpoints are both reachable, so a chord
        /// to a pinned vertex cannot make the run unconvergeable.
        double max_at_vertex = 0., max_in_edge = 0.;
        /// The same edge measure over edges with an unreachable endpoint. Reported, never gating
        /// -- the pinned twin of max_pinned.
        double max_in_edge_pinned = 0.;
        /// max |2 (Phi - c) grad Phi . n| at band vertices, reachable AND pinned.
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
    /// @param include_edge_samples false skips the edge-interior half (the expensive one).
    /// Every convergence decision passes true.
    GradientSplit gradient_split(bool include_edge_samples = true) const;

    /**
     * @brief The "energy_gradient" criterion: the front is at a critical point of Phase B's
     * energy, and every edge resolves the pull that drives it there.
     *
     * One bar for everything, B = front_conv_rel x m_front_gradient_reference:
     *  - vertices: max over the placed front vertices of ||grad F||, F the vertex's full Phase B
     *    objective (AMIPS + the two offset terms, as the shared smoother assembles it) -- the same
     *    quantity and bar as the Phase B pass stop.
     *  - edges, the interpolation test: with r(x) = (Phi(x) - c) / c on the edge's region's field,
     *    every live front edge (a, b) with midpoint m must satisfy
     *    (2 w / c) |r(m) - (r(a) + r(b)) / 2| <= B, w = 1 - w_amips. That is the second difference
     *    of the offset residual along the edge, in the units of the offset term's gradient
     *    (2 w / c) r grad Phi: how far the level set curves away from the chord, which halving the
     *    edge reduces; the bar is offset_envelope_rel x c under step_size_rel and decrement (see
     *    edge_conv_ratio()). No AMIPS at the midpoint -- a virtual split's lopsided children carry
     *    an O(w/h) AMIPS pull that never vanishes -- and no length units.
     * The sizing rule halves an edge whose endpoints pass and which fails the interpolation
     * test. dist_and_orient is still logged for information.
     */
    struct EnergyCriterion
    {
        double
            max_vertex = 0.,
            max_edge =
                0.; ///< RATIOS to the bar (1 = bar): the vertex measure per front_conv_criterion; the edge test
        double bar = 1.; ///< the ratios' bar, 1
        size_t n_vertices = 0, n_edges = 0, n_unmeasurable = 0;
        size_t n_pressed = 0, n_edges_pressed = 0; ///< skipped: pressed (see m_placement_pressed)
        size_t worst_vid = static_cast<size_t>(-1);
        Vector2d worst_edge_mid = Vector2d::Zero();
        double worst_edge_len = 0.;
        /// Reported only: the edges over the bar, split by whether both endpoints are on the level
        /// set (residual within the tube). An edge whose endpoints are on the level set and whose
        /// chord still misses it is under-resolved -- the state the vertex test cannot see; one
        /// whose endpoints are off the level set is pressed, where the sag says nothing about
        /// resolution.
        size_t n_edges_over = 0, n_edges_over_on_level = 0;
        double max_edge_on_level = 0.;
        Vector2d worst_on_level_mid = Vector2d::Zero();
        /// The single phase's states, all in the one tolerance the pipeline has, the tube
        /// half-width `tube` = offset_envelope_rel x target_distance. For a front vertex with
        /// rho = its distance from its level set and step = how far placement moved it this turn:
        ///   placed      rho <= tube
        ///   travelling  not placed, and the objective still wants to move it: its remaining
        ///               1-D Newton step along the normal, in length, is over the tube
        ///   pressed     not placed, stationary: held short of its level set by a crushed ring --
        ///               a seam or a wall. `n_pressed_touching` counts those whose ring reaches
        ///               another front, the input or a region boundary directly; the rest press
        ///               through a strip more than one cell thick.
        ///   stuck       not placed, stationary, touching nothing, with the alignment term on --
        ///               the term's own bias holding the vertex (with it off nothing but a crushed
        ///               ring can). Named in the log; never counted as converged.
        /// A front edge with both ends placed whose chord sags over the tube is `refinable`: the
        /// resolution rule refines it (refine_front_from_sag). Convergence is "placed or pressed,
        /// nothing travelling or stuck, nothing refinable": refinement and convergence read the
        /// same field, as TriWild's do.
        double tube = 0.;
        size_t n_placed = 0, n_travelling = 0, n_pressed_on = 0, n_stuck = 0;
        size_t n_pressed_touching = 0; ///< of the pressed, those touching another front or a wall
        size_t n_at_floor = 0; ///< chords over the tube whose ends are already at the sizing floor
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
        /// The alternating loop's test (and 3D's): the vertex test alone.
        bool converged() const { return vertices_ok() && n_unmeasurable == 0; }
        /// The single phase's test: the vertex test -- every front vertex's remaining Newton step
        /// along its normal at most front_conv_rel x target_distance -- and nothing left to
        /// refine. The states above are reported, not gated on: gating on "placed within the tube"
        /// makes pressed seams (whose vertices are stationary off the level set by definition) and
        /// objective-biased vertices run to the budget, doubling the front and crushing the strip.
        bool converged_single() const { return converged() && refinable.empty(); }
        double ratio() const { return bar > 0. ? std::max(max_vertex, max_edge) / bar : 0.; }
    };
    EnergyCriterion energy_criterion();
    /// Whether a front vertex touches another front, the input or a region boundary through a
    /// background triangle -- the topological fact behind the `pressed` state.
    bool front_vertex_touches_other(size_t vid) const;
    /// The edge length that would bring a front chord's sag under the tube: 3/4 L
    /// (tube / sag)^(1/p) capped at L/2, with the exponent p measured from how the level set
    /// turns across the chord (2 where it is smooth, 1 where the chord straddles a kink).
    /// See the definition; energy_criterion() and refine_front_from_sag() both use it.
    double front_chord_target(size_t va, size_t vb, double len, double sag, double tube) const;

    /// The resolution rule: sets the target length at each refinable edge's ends from
    /// front_chord_target(), graded outward. Returns the vertices changed.
    size_t refine_front_from_sag(const std::vector<EnergyCriterion::Refinable>& edges);
    /// The energy criterion as measured when the A/B loop converged; the final Phase A runs
    /// after it and the verdict must not be re-measured on that mesh.
    std::optional<EnergyCriterion> m_energy_verdict;
    /// The interpolation residual of front edge e, see EnergyCriterion. -1 when unmeasurable.
    double edge_interpolation_residual(const Tuple& e) const;

    /**
     * @brief The normal at an offset vertex. Every caller that needs one goes through here, so
     * switching the definition is a one-line edit rather than a hunt through the call sites.
     *
     * n is the unit vector from the nearest point on the input complex to the vertex -- the
     * direction the offset grew along. A property of the input geometry alone, so it does not move
     * as the offset mesh is re-triangulated and it is defined for every band vertex whether or not
     * it has live offset edges. Known weakness: it flips discontinuously across the medial axis,
     * exactly where two offset fronts approach each other.
     *
     * @return unit vector, or the zero vector where the definition cannot produce one.
     */
    Vector2d offset_vertex_normal(const size_t vid) const;


    /// Turn a residual_split()'s outside-support tally into the hard error. Separate from
    /// check_offset_within_support() so the per-round check can reuse a split it already has.
    void report_outside_support(const char* when, const DistanceSplit& s) const;
    /// Whether vid is a band vertex the optimizer could still place at target_distance.
    ///
    /// Only the domain boundary disqualifies one. m_is_on_input must not: the flag is over-broad
    /// -- splits propagate it and collapses OR it onto survivors -- so a vertex carrying it may
    /// sit a full target_distance from the complex, which is exactly where the offset wants it.
    /// check_no_vertex_on_both_surfaces() already throws on the genuinely contradictory case, so
    /// any vertex reaching here with the flag is placeable. Same rule as 3D.
    bool band_vertex_is_reachable(const size_t vid) const
    {
        // An envelope-held offset vertex is pinned: it must stay within envelope_size of the input
        // region boundary AND sit on Phi = c, target_distance away. Those are not simultaneously
        // satisfiable, so its residual gradient is a statement about the constraint rather than
        // about the offset's quality, and leaving it in max_reachable makes convergence impossible
        // by construction. Structural, not left to whether the tolerance happens to hide it.
        //
        // Still reported, never silent: the pinned half of every band measure is logged beside the
        // reachable half, the same contract vertex_is_on_domain_boundary() gets below. This is a
        // retreat, to be reverted the moment a placement that works exists.
        if (m_vertex_extra[vid].m_is_on_offset && vertex_boundary_mask(vid) != 0) return false;

        return !vertex_is_on_domain_boundary(vid);
    }

    /// Set by the placement when a vertex's last visit stopped on QualityBound, cleared when it
    /// moved. distance_criterion() counts such a vertex as placed -- its level set is
    /// unreachable by construction -- and drops edges touching one from the resolution and
    /// orientation halves; update_band_sizing_from_tolerance() does not refine such edges.
    std::vector<char> m_placement_pressed;

    /**
     * @brief TriWild's stall-driven sizing refinement, verbatim.
     *
     * Structurally TriWildMesh::refine_sizing_around_worst, down to the shared helpers in
     * wmtk/utils/SizingField.hpp and every stuck_refine_* parameter: rank faces by AMIPS,
     * force-split the worst ones' longest edges, grow the region by rings, lower the
     * per-vertex sizing scalar, grade it outward.
     *
     * Phase A only: mesh_improvement() is its one caller, and the driver only runs that as Phase
     * A. Phase B's refinement question belongs to update_band_sizing_from_tolerance().
     */
    size_t refine_sizing_around_worst(double max_metric) override;

    /**
     * @brief Why Phase A is stuck: a census of the faces stuck-refine is about to chase.
     *
     * Runs from refine_sizing_around_worst(), which only fires once max energy has stalled. The
     * question is whether refinement is even the right response, so it separates four things the
     * single MAX_ENERGY sentinel fuses:
     *
     *  - exactly inverted (is_inverted) vs merely float-degenerate (is_inverted_f only). The
     *    second is a valid triangle AMIPS2D cannot score because m_posf lost the area -- a
     *    rounding problem, not a geometry one, and splitting it makes two of them.
     *  - carrying an unrounded vertex, where m_posf is the wrong number outright.
     *  - already below the split gate, so the next pass cannot split them at all and lowering the
     *    sizing field is pure waste.
     *  - at the sizing floor, where apply_sizing_refinement has nothing left to give.
     *
     * Plus where they are: class distribution, connected clusters, and how much the set overlaps
     * the previous call's (quantised on a grid, because fids are recycled and cannot be compared
     * across passes). A high overlap with a low cluster count says the pass is chasing the same
     * few spots forever; a scattered, changing set says something is manufacturing new
     * degeneracies as fast as they are refined.
     */
    void log_stuck_refine_census(double max_metric, double filter_energy);

    /**
     * @brief For every element above `filter_energy`, why its edges cannot be split.
     *
     * log_stuck_refine_census() answers "what are the bad elements"; this answers "what is
     * stopping the mesh from fixing them", the question that matters when Phase A refines
     * somewhere else instead. Attributes each of a bad face's three edges to the first gate that
     * refuses it, in the order the code applies them (TriOptimizerMeshSplit.cpp):
     *
     *   short     length^2 < splitting_l2 * mean(sizing)^2 -- never even offered to the queue.
     *             The remedy is the sizing field, not the split.
     *   valence   a link vertex is over split_high_valence_threshold. Reported as a ceiling: the
     *             real gate is one such split per vertex per pass, which a static probe cannot
     *             see, so this counts vertices that could be refused, not that were.
     *   contain   the dispatched envelope refuses one of the two halves, and the column says
     *             which envelope.
     *   free      nothing blocks it -- so a face all of whose edges are `free` is starved by no
     *             gate, and the stall is elsewhere.
     *
     * Two shortcuts, both stated so the output is not over-read:
     *
     *  - the midpoint cannot invert a healthy parent: each child has exactly half the parent's
     *    signed area, so the base's exact inversion check can only fire on an already-inverted
     *    parent. The census reports the parent's own inversion instead of probing.
     *  - the envelope of a child segment is the parent's: surface_envelope_for_edge dispatches on
     *    edge_mask(), and the midpoint's mask is itself the AND of the parent's endpoints, so
     *    mask(a,m) == mask(m,b) == mask(a,b) and one dispatch serves both halves. Same reasoning
     *    split_adjust_position relies on.
     *
     * Each bad face is also located: centroid, distance to the input complex, and Phi/c there,
     * which separates a collided corridor between two fronts from somewhere in the background.
     *
     * Diagnostic only: reads the mesh, writes only the log.
     */
    void log_refine_block_census(const std::string& when, double filter_energy) const;

    /**
     * @brief Instrumentation only: which operation manufactures the MAX_ENERGY needles.
     *
     * The base leaves two doors open and this counts what goes through each.
     *
     *  - a split is never refused on quality (it checks orientation, rounding and containment
     *    only), so needles counted at a freshly split midpoint were created by that split.
     *  - a collapse is admitted by collapse_quality_allowed() when `q <= ring_max`, and once a
     *    single needle sits in the ring ring_max is MAX_ENERGY -- so the clause admits a collapse
     *    that produces another needle. The counters below say which clause did the admitting.
     */
    bool collapse_quality_allowed(size_t v1, size_t v2, double q, double ring_max) const override;

    mutable std::atomic<size_t> m_deg_split_created{0};
    mutable std::atomic<size_t> m_deg_collapse_offered{0};
    mutable std::atomic<size_t> m_deg_collapse_allowed{0};
    mutable std::atomic<size_t> m_deg_collapse_by_ringmax{0};
    mutable std::atomic<size_t> m_deg_collapse_by_stop{0};
    mutable std::atomic<size_t> m_deg_collapse_by_unrounded{0};
    /// Values at the previous census, so each census can report deltas rather than totals.
    std::array<size_t, 6> m_deg_prev_counts{{0, 0, 0, 0, 0, 0}};

    /**
     * @brief Where the first needles come from -- a tripwire, not a census.
     *
     * The census counts the population once it exists and the attribution counters say which
     * operation touches them; neither says how the first one is born. This logs the first
     * kNeedleReports needle faces any operation hook sees, with what tells a creation from a copy:
     * the operation, the parent quality where there is one, both the float and the exact
     * orientation, full-precision coordinates, and each vertex's flags, birth epoch and rounding.
     *
     * Deliberately capped -- once the force-split loop engages there are thousands per pass, and
     * it is the first few that carry the information.
     */
    void report_needle(const char* op, size_t fid, double parent_q) const;
    static constexpr size_t kNeedleReports = 12;
    /**
     * @brief What counts as a needle for the tripwire -- deliberately far below MAX_ENERGY.
     *
     * A healthy triangle is O(2); 1e6 is far outside anything the optimizer should tolerate and
     * far below the sentinel, so the creation event is caught while its parent is still scoreable
     * and can be quoted. A `>= MAX_ENERGY` test misses parents that are already catastrophically
     * flat, which is where the collinearity actually originates.
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

    /// TriWild's bare collapse passes are off for the offset: the opening one (length gate off),
    /// the closing one, and coarsen_mesh(). The opening pass is meant to run once on an inserted
    /// input, but the A/B loop calls mesh_improvement() every round, so with no length gate the
    /// quality test alone demolishes the band interior and background each round. The sizing field
    /// cannot refuse a collapse -- only the length gate can, and that pass switches it off -- and
    /// the offset envelope holds the boundary, not the interior. Phase A is otherwise TriWild's
    /// mesh_improvement(), untouched.
    bool optimization_bare_coarsen_passes() const override { return false; }

    /**
     * @brief A collapse is accepted by the same criterion the smoothing minimises.
     *
     * The smoother places an offset vertex by minimising w (Phi - c)^2, so every other operation
     * must answer to that same measure or it undoes in one collapse what the smoother spent an
     * iteration achieving. A length gate cannot express it: it asks whether an edge is short
     * against a sizing target, a statement about the mesh, not whether the boundary is still the
     * offset, a statement about the geometry -- and only the second is what the run is for.
     */
    bool collapse_edge_after(const Tuple& t) override;
    /// Max of the two normalized criteria (AMIPS over stop, residual over tolerance) on this
    /// face; >= 1 means it fails at least one. The coarsen-mode collapse accept reads it, and
    /// it is the per-face form of optimization_quality_stats()'s Phase B max.
    double face_criterion_rel(const size_t fid) const;
    /**
     * @brief Put the frames beside the run's own output, and rename them into one timeline.
     *
     * Every debug frame in 2D goes through here -- the shared driver's per-pass frames, Phase B's,
     * and the A/B driver's per-phase ones -- so this is the one place that can give them all a
     * common order. A bare `debug_N` or `phase_<r><A|B>` comes out as
     *
     *     <output>_step_<NNNNN>_r<round><A|B><pass>.vtu      a pass inside a phase
     *     <output>_step_<NNNNN>_r<round><A|B>_end.vtu         the frame the phase handed on
     *
     * NNNNN is a single monotonic counter across the whole run, so sorting on it is run order.
     * <pass> counts passes within the current phase and restarts whenever the round or the phase
     * changes, so `r1A3` reads as "round 1, phase A, third pass"; the restart is detected here, so
     * no call site has to remember to reset anything. Round 0 is construction.
     *
     * Renaming here rather than at the call sites is what keeps this out of the shared driver:
     * wmtk::TriOptimizerMesh is also triwild's and simwild's, so its naming is not ours to change.
     */
    void write_smoothing_debug_output(const std::string& path) const override
    {
        const char ph = (m_phase == OptPhase::A) ? 'A' : (m_phase == OptPhase::B ? 'B' : 'S');
        if (m_ab_round != m_debug_last_round || ph != m_debug_last_phase) {
            m_debug_last_round = m_ab_round;
            m_debug_last_phase = ph;
            m_debug_pass = 0;
        }
        // What this frame is, as a compact token: round, phase, pass within the phase, and the
        // operation it follows (from m_debug_pass_name -- the shared driver writes several frames
        // per operation group). Phase B and the single phase write one per sweep.
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
        // The file name is the sequence number and nothing else, so ParaView groups the frames into
        // one time series: it needs the digits last, right before the extension, and every frame
        // of a series to share its prefix. The label goes to <output>_frames.txt, one
        // "NNNNN<tab>label" line per frame, which is what the polyscope viewer reads for its
        // slider.
        const size_t idx = m_debug_seq++;
        append_frame_label(idx, label);
        const_cast<TopoOffsetTriMesh*>(this)->write_vtu(
            m_offset_params.output_path + fmt::format("_{:05d}", idx));
    }

    /// One line of <output>_frames.txt; truncates the file on the first frame. See
    /// write_smoothing_debug_output().
    void append_frame_label(size_t idx, const std::string& label) const;

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
        const std::vector<std::string>& tag_names,
        const std::string& curve_name = "");

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
     * Must be called after init_from_image(...) and label_input_complex(). The potential is built
     * from the boundary of the complex rather than its interior: Phi's 2D primitives are segments
     * and points, so a solid input region enters as its outline. Outside the region -- the only
     * place an offset exists -- the two descriptions agree exactly.
     */
    void init_input_complex_bvh();

    /**
     * @brief Build the smooth offset potential from the extraction init_input_complex_bvh() kept.
     *
     * Separate from that call only because it needs target_distance and offset_dhat_factor, which
     * a caller wanting nothing but the distance field has no reason to have set. The geometry is
     * still extracted exactly once, so the potential and the BVH cannot describe different inputs.
     */
    void init_offset_potential();

    /// The complex as the potential sees it: vertices, its boundary segments, and its isolated
    /// points. Filled by init_input_complex_bvh(), consumed by init_offset_potential().
    MatrixXd m_phi_V;
    MatrixXi m_phi_E;
    MatrixXi m_phi_F; ///< the complex faces, in the same vertex index space (for per-region BVHs)
    std::vector<int> m_phi_P;

    //// overriden splits/invariants
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
    bool invariants(const std::vector<Tuple>& tris) override;
    //// overriden splits/invariants

    /**
     * @brief TriWild over the input mesh, before any of the offset exists.
     *
     * Runs the shared mesh_improvement() with Phase A's own parameters and units, at a point where
     * the only tracked surfaces are the tag-region boundaries (input complex and domain wall among
     * them) and the only containment is their per-tag envelopes. There is no offset yet, so no
     * offset envelope and no Phi term: this is TriWild, exactly.
     *
     * Worth a pass because marching_tris() puts the offset boundary on the background
     * triangulation's own cell boundaries: how far the constructed offset lands from the complex
     * is a property of the input mesh, and init_offset_potential() sizes dhat from that reach. One
     * coarse cell touching the complex inflates dhat several-fold, and a large dhat merges the
     * level sets of features that are close together, which no later stage can undo.
     *
     * The sizing field is seeded first, to target_distance on the input-complex boundary and
     * graded outward: a band built on cells of delta scale lands near delta from the complex,
     * which is what keeps dhat small. Unseeded, the base field is m_params.l everywhere, far
     * coarser than the mesh around the complex, so the pass would coarsen the input instead.
     */
    void pre_optimize_input_mesh();

    void execute_offset(const std::filesystem::path& output_file);

    /**
     * @brief execute simplistic marching tris. All edges with one vertex labelled 0 and the other
     * 1/2 are split, always at the midpoint.
     *
     * No target_distance enters construction at all: the paper places inserted vertices at the
     * midpoint (Sec. 5.2) and leaves the distance to Step 3, so carrying the boundary out to the
     * level set is entirely the optimization phase's job. Same as 3D.
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
     * The offset is a level set of a field defined everywhere and the output mesh only samples
     * that field along one curve, so a result that looks wrong cannot be diagnosed from the mesh
     * alone. This writes the field itself: Phi (clamped, since it diverges on the input complex),
     * the residual as a length, and the exact Euclidean distance beside it, all as vertex fields
     * on a triangulated grid so a viewer can draw the isoline Phi = c directly.
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
