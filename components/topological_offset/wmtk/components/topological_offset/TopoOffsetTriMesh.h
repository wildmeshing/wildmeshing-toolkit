#pragma once
#include <wmtk/TriMesh.h>
#include <wmtk/TriOptimizerMesh.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
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

    /**
     * @brief Which tag boundaries this vertex lies on -- one bit per input tag, ambient included.
     * See TopoOffsetTriMesh::m_tag_envelopes for what the bits dispatch to.
     *
     * Seeded in init_surfaces_and_boundaries() from the input partition, then propagated by the
     * operations: a split's new vertex takes the AND of its endpoints (it lies on a boundary only
     * if the whole edge did), a collapse's survivor the OR (it carries both vertices' geometry).
     */
    uint64_t m_boundary_mask = 0;

    /// Which split pass created this vertex, from wmtk::TriOptimizerMesh::m_op_epoch; 0 means not
    /// created by an optimization split. Read only by the needle diagnostics' per-vertex lines.
    /// Assigned at each split, never OR'd -- a recycled slot carries a dead vertex's epoch.
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
        SphereTrace = 1, // marching_tris (construction_mode): sphere tracing along the edge to
                         // d(x) = m_construction_distance, midpoint when the trace leaves the
                         // edge
        Optimization = 2 // the optimization phase; the shared engine places the vertex
    };

public:
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
     * init_input_complex_bvh() has one call site, before construct_offset() runs, so this holds the
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
    /// Rebuild m_*_region from the mesh. `log` false suppresses the per-turn "[regions]" line:
    /// write_vtu() re-derives the map for its frame diagnostics and puts the old one back.
    void assign_band_regions(bool log = true);
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
     * for the whole run, m_offset_envelope is built for the final pass.
     * rebuild_offset_envelope() clears this and must keep doing so. Guarded by m_isect_mutex.
     */
    mutable std::map<uint64_t, std::shared_ptr<SampleEnvelope>> m_offset_isect_cache;

    /**
     * @brief The containment a simplex with this region mask, on/off the offset front, must
     * satisfy -- the intersection of everything that holds it, or null if nothing does.
     *
     * The single place the two containment families are composed. `region_mask` dispatches
     * through envelope_for_mask() (itself an intersection when the mask is multi-bit, which is
     * what pins a junction to the junction); `on_offset` adds m_offset_envelope, in the
     * frozen-front final pass only. Every operation's containment check reaches this through
     * surface_envelope_for_edge(), so in the final pass split, collapse and swap hold the offset
     * boundary to its tube; in the loop they do not. The smoother asks with `on_offset` false
     * (smoothing_containment_envelope()): placing the front is what moves the offset boundary,
     * and a tube around where it currently sits would cap how far it can travel. As in 3D.
     */
    std::shared_ptr<SampleEnvelope> containment_for(uint64_t region_mask, bool on_offset) const;

    /// The final pass: front vertices are not smoothed (see smooth_before()).
    bool m_freeze_front = false;

    /**
     * @brief Which boundaries the region-class envelopes hold, and how they are built.
     *
     * PerTag (deform_others false): one exact tube per input tag around that tag's boundary
     * segments, the domain wall in the tags of its wall faces. A vertex carries the bit of every
     * tube it lies on and is contained in their intersection, so every region boundary -- the
     * input complex and the wall included -- is held.
     *
     * WallComplex (deform_others true): exactly two tubes, the domain wall and the boundary of
     * the input complex (any dimension, any manifoldness: it is a set of segments), under the
     * pseudo-tags m_wall_tag / m_complex_tag. Every other region boundary carries no bit and is
     * held by nothing; the medium around it is plastic, see face_is_plastic().
     *
     * Either way build_boundary_envelopes() derives the masks and tubes from the mesh as it
     * stands when called: at load (PerTag, before the complex is labelled), when deform_others
     * switches the setup at construction, and fresh at the start of the final pass. The offset
     * tube is separate and unchanged.
     */
    enum class EnvelopeSetup { PerTag, WallComplex };
    EnvelopeSetup envelope_setup() const
    {
        return m_offset_params.deform_others ? EnvelopeSetup::WallComplex : EnvelopeSetup::PerTag;
    }
    static constexpr int64_t m_wall_tag = -2; ///< pseudo-tag: the domain wall's tube
    static constexpr int64_t m_complex_tag = -3; ///< pseudo-tag: the input complex boundary
    /// The name a tag or pseudo-tag prints under.
    std::string envelope_key_name(int64_t tag) const;
    /// Whether this edge lies on the boundary of the input complex: exactly one incident face
    /// carries label 1, or the edge itself does while neither face does (a curve or edge piece).
    bool edge_is_complex_boundary(const Tuple& e) const;
    /// Rebuild every region-class tube and every vertex's boundary mask from the current mesh
    /// under `setup`. PerTag at load (the complex is not labelled yet), WallComplex when
    /// deform_others switches it at
    /// construction, envelope_setup() fresh at the final pass. The tracked-edge flags are left
    /// alone: they are the topology the operations maintain. `when` labels the log line.
    void build_boundary_envelopes(const char* when, EnvelopeSetup setup);

    /**
     * @brief The tube the offset boundary may not leave during the frozen-front final pass, of
     * half-width offset_envelope, built from the boundary as the loop left it just before that
     * pass (containment_for() holds the final pass's operations to it). Null until then: the
     * loop holds the offset boundary to no envelope. Unlike m_tag_envelopes, which must never be
     * rebuilt. As in 3D.
     */
    std::shared_ptr<SampleEnvelope> m_offset_envelope;

    /// Rebuild m_offset_envelope from the current offset-boundary segments, and drop the
    /// intersections memoized against the old one; also refresh_released_envelope().
    void rebuild_offset_envelope();
    /// Rebuild deform_others' released-boundary tube now, between passes (see
    /// released_envelope(), which never rebuilds mid-operation).
    void refresh_released_envelope();

    /// Hard error if any vertex is on both the input complex and the offset boundary -- a state
    /// no placement satisfies. Called at construction.
    void check_no_vertex_on_both_surfaces(const char* when) const;

    /// TriWild's loop, the front placed inside its smoothing passes. `final_stage` false is the
    /// init_optimize loop: it returns on convergence without the frozen-front final pass and
    /// without deciding the verdict. `refine` false skips the halving. `label` names the loop in
    /// its log. As in 3D.
    void optimize_offset_loop(
        bool final_stage = true,
        bool refine = true,
        const std::string& label = std::string());

    /// init_optimize: optimize_offset() opens with a stencil_order loop without refinement. Set
    /// by marching_tris(), only when the target is beyond the maximum marchable distance.
    bool m_init_optimize = false;
    /// Leads every debug frame label: i during the init_optimize loop, empty otherwise.
    std::string m_frame_prefix;

    /// Max over the front vertices of ||grad F . n||, F the vertex's front objective
    /// (front_objective()). Logged as the loop's gradient reference.
    double front_gradient_linf();
    /// Its value on the band as constructed, measured once before turn 1.
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
     * @brief THE per-cell energy, read from the mesh as it is -- labels, neighbours and positions,
     * nothing hypothetical. The 2D twin of TopoOffsetTetMesh::tet_energy().
     *
     *     E(t) = w A(t) + [t is band] * sum over the live front chords e of t of O(e)
     *     O(e) = (1/N) sum_i (relative_residual(q_i) / front_conv_frac())^2
     *
     * A(t) is the base's TriOptimizerMesh::get_quality(), the AMIPS2D the engine stores as the
     * face quality (3D stores AMIPS^3 there, which is why its energy reads AMIPS^3); its
     * MAX_ENERGY (unscoreable) passes through unchanged, and w is offset_amips_weight
     * (weighted_amips()). q_i are the N points of e's stencil_order stencil
     * (for_each_edge_sample()) on the field of e's band face (potential_for_face()), e's ends
     * sorted so that a chord's term does not depend on which face or which operation reads it. O is
     * edge_offset_term(), the squared chord measure in units of the tolerance, 1 at the bar. With
     * no field yet (m_offset_potential null) E = w A.
     *
     * An unmeasurable chord makes the face MAX_ENERGY, the engine's own "unscoreable". THE BAND
     * SIDE CARRIES THE CHORD: only a band face (face_is_offset_band()) adds terms, and a live front
     * chord has exactly one band side (edge_is_offset_surface_live()), so every front chord is
     * counted once.
     *
     * WHERE IT IS COMPARED: the collapse rule (collapse_before_vertex() / collapse_edge_after(),
     * early half collapse_quality_allowed()), the swap rule (swap_edge_before() /
     * swap_edge_after(), early half swap_quality_allowed()), the smoother's projected step and
     * veto (smoothing_cell_energy()) and the front and interior vetoes. The engine's stored face
     * quality stays AMIPS2D and every engine diagnostic reads it unchanged.
     */
    double tri_energy(size_t fid) const;
    /// tri_energy() with the face's AMIPS given, as the shared smoother has just computed it.
    double tri_energy(size_t fid, double amips) const;
    /// The shared smoother's per-face comparison -- its projected step for a vertex held on a
    /// surface, and its quality veto -- is the per-cell energy, so every smoothing test judges a
    /// move by the energy the smoother minimises (smoothing_extra_energy()). As in 3D.
    double smoothing_cell_energy(const size_t fid, const double quality) const override
    {
        return tri_energy(fid, quality);
    }
    /// The AMIPS part of the energy: offset_amips_weight times AMIPS, the MAX_ENERGY sentinel
    /// (unscoreable) passed through unscaled so that it stays the largest value anywhere.
    double weighted_amips(const double amips) const
    {
        return amips >= MAX_ENERGY ? amips : m_offset_params.offset_amips_weight * amips;
    }
    /// Max of tri_energy() over `fids` (0 for none): the number every rule above compares.
    double max_tri_energy(const std::vector<size_t>& fids) const;
    /// 1 / front_conv_frac()^2: the factor that turns a squared relative error into a squared
    /// error in units of the tolerance. The weight of the front smoother's offset terms, and the
    /// scale of edge_offset_term(). As in 3D.
    double offset_term_weight() const
    {
        const double f = m_offset_params.front_conv_frac();
        return 1. / (f * f);
    }

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

    /// {max_dist_err, avg_dist_err, max_phi_residual, avg_phi_residual, max_grad, avg_grad,
    /// max_grad_at_vertex, max_grad_in_edge}. max_grad is the convergence criterion -- the full
    /// placement-gradient norm at band vertices -- so max_grad_at_vertex repeats it and
    /// max_grad_in_edge is the chord diagnostic; the rest are diagnostics. One entry for the whole
    /// run, as in 3D.
    std::vector<std::array<double, 8>> optimization_metrics;
    /// The turn the run is in, 1-based; 0 before the loop starts. Read only by
    /// write_smoothing_debug_output(), to tag each frame with the turn it belongs to.
    int m_round = 0;
    /// Monotonic frame counter for the debug timeline. Mutable because the write hook is const.
    mutable size_t m_debug_seq = 0;
    /// DEBUG_output: the label of each debug frame, indexed by its sequence number, and, per
    /// companion suffix, the frame indices that actually produced one. Both exist only to write
    /// the ParaView collections -- see write_debug_pvd(). Same in 3D.
    mutable std::vector<std::string> m_debug_frame_labels;
    mutable std::map<std::string, std::vector<size_t>> m_debug_pvd_series;
    /// DEBUG_output: rewrite <output>{_main,_off,_surf,_edge,_front}.pvd, a ParaView time
    /// series over the debug frames. Needed because ParaView only groups a file series when the
    /// index is immediately before the extension, which is false for every companion
    /// (<output>_NNNNN_off.vtu). Called after every frame, so a killed run still opens.
    void write_debug_pvd() const;
    /// Pass index within the current turn, and the (turn, loop-or-final-pass tag, prefix) it
    /// belongs to -- when those change the index restarts. All exist only to name frames.
    mutable int m_debug_pass = 0;
    mutable int m_debug_last_round = -1;
    mutable char m_debug_last_tag = '?';
    mutable std::string m_debug_last_prefix = "?";
    /// The run's verdict: the front resolved (EnergyCriterion::converged(): every front chord's
    /// measure within the bar, nothing unmeasurable) AND the final quality under stop_energy.
    /// Read by the report and by throw_on_nonconvergence.
    bool m_converged = false;
    /// The finishing-pass half of the verdict: max AMIPS < stop_energy once the front is resolved,
    /// after the final pass when one ran. True when no pass was needed; false when the pass ended
    /// still over. m_quality_max_amips is the value it was judged on.
    bool m_quality_converged = true;
    double m_quality_max_amips = 0.;

    std::atomic<int> iter_cnt_split = 0, iter_cnt_collapse = 0, iter_cnt_swap = 0;
    std::atomic<int> iter_cnt_collapse_offset_removed{0};
    /// Operations refused because they would have left an offset-boundary chord over tolerance.
    std::atomic<int> iter_cnt_collapse_offset_reject{0};
    std::atomic<int> iter_cnt_swap_offset_reject{0};
    /// The energy rules (see tri_energy()): collapses refused for raising the max energy over
    /// the survivor's ring, swaps for not strictly lowering it over the faces they make.
    mutable std::atomic<int> iter_cnt_collapse_energy_reject{0}; // both halves count here
    mutable std::atomic<int> iter_cnt_swap_energy_reject{0};
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
     * @brief Construction placement in the marching: sphere tracing along the edge from p_in (the
     * endpoint in the input complex, label != 0) towards p_out (the background endpoint) for the
     * point where d(x) = D, D = m_construction_distance and d(x) the distance to the input
     * complex through m_input_complex_bvh. From t = 0 the trace evaluates d at the current
     * point and steps forward by D - d, the largest step that cannot cross the level set (d is
     * 1-Lipschitz); it stops when |d - D| <= sphere_trace_target_rel_tol x D and returns true
     * with p_new there. It returns
     * false, p_new untouched, as soon as the current point reaches or passes p_out (t >= L: the
     * level set is not on the edge) or would move behind p_in (d(p_in) already beyond the target);
     * the caller then places the plain midpoint. Every step taken is longer than the tolerance, so
     * the trace ends within L / (tol x D) steps; `steps` returns how many it took.
     * No snapping away from the endpoints: a point found arbitrarily close to p_out is used as is.
     * Same as 3D.
     */
    bool edge_split_sphere_trace(
        const Vector2d& p_in,
        const Vector2d& p_out,
        Vector2d& p_new,
        size_t& steps) const;
    /// marching_tris() tallies for the construction log: edges placed on the level set / at the
    /// midpoint because the trace left the edge, and the trace steps (total, max). Reset at the
    /// start of marching_tris().
    size_t m_marching_root_splits = 0, m_marching_midpoint_splits = 0;
    size_t m_marching_trace_steps = 0, m_marching_trace_steps_max = 0;
    /// The distance the marching's sphere trace aims for: target_distance, or half the maximum
    /// marchable distance under construction_mode "max_marchable_fallback" (see the marching).
    double m_construction_distance = 0.;

    /**
     * @brief Reject any collapse that violates the substructure link condition, and remember the
     * survivor's sizing for sizing_collapse_min = false.
     *
     * The base applies the link condition only when both endpoints already sit on a tracked
     * surface or the bbox, which is the right rule for tetwild and simwild but not here: the
     * offset region is a thin band, and a collapse with only one endpoint on the boundary can
     * still pinch its two sides together and make the region non-manifold. The offset asks
     * unconditionally. The energy rule's before-half is taken in collapse_before_vertex(), which
     * the base calls before its scoring loop. As in 3D.
     */
    bool collapse_edge_before(const Tuple& t) override;
    /**
     * @brief THE ENERGY RULE for a collapse (see tri_energy()), on the real mesh once the
     * connectivity is committed, then the base's after-hook, the coarsening bar, the sizing
     * restore and the rest re-stamp. A refusal returns false, which the engine rolls back.
     *
     * The 3D twin applies the rule in collapse_after_connectivity(), a hook the 2D engine does
     * not have; here it runs as the first thing in the after-hook, BEFORE
     * TriOptimizerMesh::collapse_edge_after(), whose last step (collapse_after_vertex()) counts
     * the collapse as done. A collapse moves no vertex, so the energy is read from the mesh as it
     * now is.
     */
    bool collapse_edge_after(const Tuple& t) override;

    /**
     * @brief Reject a flip whose new edge already exists, and cache the energy rule's
     * before-half.
     *
     * Flipping (a,b) to (c,d) when c and d are already joined creates a second edge between the
     * same pair. Across a thin offset band that is how the two sides get stitched together and the
     * region stops being manifold. The base refuses tracked-surface edges but not this.
     */
    bool swap_edge_before(const Tuple& t) override;

    /// THE ENERGY RULE for a swap (see tri_energy()): the max of tri_energy() over the two faces
    /// the flip made must be STRICTLY below the max over the two it replaced
    /// (m_swap_energy_before). The 3D twin is swap_after_cells().
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
     * @brief What smoothing did with each class of vertex, per pass. Same fields as 3D.
     *
     * The base's SmoothRejectCounters says why a move was refused; it cannot say what kind of
     * vertex was asking. What is worth counting here is the dispatch: how many attempts were on
     * the offset boundary (the ones carrying the offset term), how many on another region's
     * boundary, and how many were turned away before the smoother saw them at all.
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
        }
    };
    SmoothTrace m_smooth_trace;

    /// How the solves this class makes itself ended, per smoothing pass. Background: every
    /// smooth_nonfront_vertex() solve -- the 3D engine counts these itself (its m_newton), the
    /// 2D engine does not, so the component keeps the counter. Front: every front placement.
    /// Plastic: the rest-shape solve of smooth_plastic_vertex(). Logged and reset by
    /// log_smoothing_pass_accounting().
    ///
    /// One difference from 3D, read before comparing the two logs: the 2D engine's smoother
    /// (optimization::smooth_vertex_2d) catches a solver throw internally and reports nothing,
    /// so the `threw` count is always 0 here; a solve that threw shows only through its stop
    /// status (LineSearchFailed, the throw polysolve raises).
    optimization::NewtonCounters m_newton;
    optimization::NewtonCounters m_newton_front;
    /**
     * @brief DEBUG_output only: every front solve since the last debug frame, so the frames
     * show per vertex what m_newton_front counts per pass (frame fields front_newton_iters and
     * front_newton_status, see write_vtu()). Appended by smooth_front_vertex(), emptied by
     * write_debug_frame() once the frame is written. As in 3D.
     */
    struct FrontSolveRecord
    {
        size_t vid;
        int iterations;
        int status; ///< polysolve's stop status + 1: NewtonCounters::status_name()'s numbering
    };
    std::vector<FrontSolveRecord> m_front_solve_log;
    std::mutex m_front_solve_log_mutex;
    /// The Newton stopping rule every smoother of this component runs with, set on the thread's
    /// shared solver (the engine's m_solver slot) by smoothing_solver() at the start of every
    /// visit: relative gradient tolerance 1e-6 beside the engine's absolute 1e-10. See the 3D
    /// twin for the measurement behind it.
    static constexpr double kSmoothRelGradNormTol = 1e-6;
    /// The thread's shared solver, created with the engine's parameters if needed, with
    /// kSmoothRelGradNormTol applied. Every smoothing path of this component takes it from here.
    polysolve::nonlinear::Solver& smoothing_solver();
    /// One smooth_vertex_2d() call with its solve recorded in `newton`: the 3D engine's
    /// smooth_vertex_3d() takes a NewtonCounters and the 2D one does not, so the component
    /// records the solve itself. A solve runs exactly when the ring is not already inverted in
    /// floats (smooth_vertex_2d()'s only refusal before solving; two_stage is off on every
    /// component path), so `solved` reports that, and the solver still holds its state.
    bool smooth_vertex_2d_counted(
        const Tuple& t,
        const optimization::SmoothVertexOptions& opts,
        optimization::NewtonCounters& newton,
        bool* solved = nullptr);
    /// The front veto (smooth_front_vertex()): moves whose Newton solve succeeded and reached
    /// the veto, and how many it refused for raising the ring's max tri_energy. Reported and
    /// reset per pass beside the Newton counters.
    std::atomic<size_t> m_front_veto_asked{0}, m_front_veto_fired{0};
    /// The repulsion passes and rounds (repulsion_smoothing()), set only while they run: the
    /// Euclidean field at level 2 x target_distance. Null otherwise, so every other path is
    /// untouched.
    std::shared_ptr<const OffsetPotential2D> m_repulsion_potential;
    /// Per repulsion pass or round, as m_newton_front and the front veto counters are per loop
    /// pass.
    optimization::NewtonCounters m_newton_repulsion;
    std::atomic<size_t> m_repulsion_veto_asked{0}, m_repulsion_veto_fired{0};
    /// The edges marching_tris() splits: exactly one end in the input complex (label != 0).
    bool is_marched_edge(const size_t a, const size_t b) const
    {
        return (m_vertex_extra[a].label == 0) != (m_vertex_extra[b].label == 0);
    }
    /// While the repulsion runs: an outer end of a marched edge -- off the complex, with a
    /// neighbour on it -- that no envelope holds. Asked of the mesh every time, so it follows the
    /// repulsion rounds' operations. As in 3D.
    bool is_repulsion_vertex(const size_t vid) const
    {
        if (!m_repulsion_potential || m_vertex_extra[vid].label != 0 ||
            vertex_boundary_mask(vid) != 0) {
            return false;
        }
        for (const size_t u : get_one_ring_vids_for_vertex_duplicate(vid)) {
            if (m_vertex_extra[u].label != 0) return true;
        }
        return false;
    }
    /// O(v) = (max(0, 2 delta - d(v)) / front_conv)^2: the Euclidean residual against level
    /// 2 delta is (d - 2 delta) / (2 delta), so the weight (2 delta / front_conv)^2 =
    /// 4 offset_term_weight() puts it in units of the tolerance, as the front's terms are. One
    /// object for the whole repulsion (value() changes nothing in it, so threads share it); set
    /// with m_repulsion_potential.
    std::shared_ptr<OffsetEnergy2D> m_repulsion_term;
    double repulsion_term_at(const size_t vid) const
    {
        Eigen::VectorXd x = m_vertex_attribute[vid].m_posf;
        return m_repulsion_term->value(x);
    }
    /// THE REPULSION PART OF THE PER-TRI ENERGY, before the march (tri_energy()): a face carries
    /// O(v) of each of its corners v that is an outer end within it -- off the complex, held by
    /// no envelope, in a face that has a complex vertex, i.e. joined to the complex by a marched
    /// edge of this face. These faces are the band the march will make, so this is the
    /// counterpart of a band face carrying its front chord's term. Reads the face's own three
    /// vertices only. 0 while the repulsion does not run. As in 3D.
    double repulsion_cell_term(const std::array<size_t, 3>& vids) const
    {
        if (!m_repulsion_potential) return 0.;
        bool has_complex = false;
        for (const size_t v : vids) has_complex = has_complex || m_vertex_extra[v].label != 0;
        if (!has_complex) return 0.;
        double e = 0.;
        for (const size_t v : vids) {
            if (m_vertex_extra[v].label == 0 && vertex_boundary_mask(v) == 0) {
                e += repulsion_term_at(v);
            }
        }
        return e;
    }
    /// How many faces of vid's ring carry its term (repulsion_cell_term()): the smoother's
    /// objective is the ring's sum of the per-tri energy, so it charges O(v) that many times.
    size_t repulsion_cells_at(const size_t vid) const
    {
        size_t n = 0;
        for (const size_t fid : get_one_ring_fids_for_vertex(vid)) {
            for (const size_t u : oriented_tri_vids(fid)) {
                if (m_vertex_extra[u].label != 0) {
                    ++n;
                    break;
                }
            }
        }
        return n;
    }
    /// O(v) for vid's smoothing objective, charged once per carrying face.
    std::shared_ptr<OffsetEnergy2D> repulsion_energy(const size_t vid) const
    {
        return std::make_shared<OffsetEnergy2D>(
            m_repulsion_potential,
            double(repulsion_cells_at(vid)) * 4. * offset_term_weight(),
            true,
            true,
            /*one_sided=*/true);
    }
    /// repulsion_rounds: whether the complex is still simplicially embedded around the faces
    /// `fids` an operation just made -- no face off the complex has complex vertices spanning an
    /// edge off the complex (tri_is_simp_emb()'s test, decided from the faces' labels, which the
    /// operations carry, instead of the edge labels, which they do not). A collapse or a swap
    /// that breaks it is refused. Defined in TopoOffsetTriMesh.cpp. As in 3D.
    bool repulsion_embedding_kept(const std::vector<size_t>& fids) const;
    /// Whether the edge (a, b) is in the input complex, from the faces' labels: an edge of a face
    /// of the complex, or, for offset_in, a domain-boundary edge of a body face. Single-body
    /// offset_in or offset_out only, which repulsion_smoothing() checks. The 3D twin also takes a
    /// face.
    bool simplex_in_input_complex(size_t a, size_t b) const;
    std::atomic<size_t> m_repulsion_embed_refused{0};
    /// Diagnostic: where the front solves stop. Per pass, histograms of the final gradient norm
    /// of each front solve (log10 bins, -14..+7) and of its ratio to the solve's first gradient
    /// norm, read from polysolve's Criteria after the solve. As in 3D.
    static constexpr int kGradBins = 22;
    std::array<std::atomic<size_t>, kGradBins> m_front_grad_abs{}, m_front_grad_rel{};
    optimization::NewtonCounters m_newton_plastic;

    /// DEBUG_crossings (log-only). The ring measure of every front vertex as the last pass left
    /// it (NaN where a vertex has none), the positions it was taken at, and whether it is current
    /// (consolidation renumbers vertices, so the loop clears it). A pass's line counts the front
    /// vertices whose ring measure went from <= 1 to > 1 (up), back (down), new vertices already
    /// over (a split's, or an old id at a new position after a storage retry), and vertices over
    /// the bar that the pass removed. As in 3D.
    std::vector<double> m_cross_ring;
    std::vector<Vector2d> m_cross_pos;
    bool m_cross_valid = false;
    /// Per smoothing pass: accepted front moves that took the moved vertex itself (own), or one of
    /// its front neighbours (neighbour), from a ring measure <= 1 to > 1.
    std::atomic<size_t> m_cross_own{0}, m_cross_neighbour{0};
    /// The exit test's ring measure of every vertex (energy_criterion()), chord for chord.
    std::vector<double> front_ring_measures() const;
    /// One vertex's ring measure, from its own live front chords (NaN if it has none).
    double ring_measure_at(size_t vid) const;
    /// Take the snapshot; with `compare`, log the crossings against the previous one first.
    /// `match_positions`: an id counts as the same vertex only at the same position (operation
    /// passes, which move no vertex; a storage retry inside one renumbers).
    void crossing_snapshot(const std::string& pass, bool compare, bool match_positions);
    /// The engine calls this at the start of local_operations() and after each operation pass
    /// that ran; DEBUG_crossings hooks the operation passes here. Log-only.
    void update_attributes() override;

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
    /// The collapse energy rule's before-half: the max of tri_energy() over the one-rings of v1
    /// and v2, cached by collapse_before_vertex() and compared by collapse_edge_after().
    mutable wmtk::threading::enumerable_thread_specific<double> m_collapse_energy_before;
    /// offset_collapse_changed_cells: the collapse in flight's before-energies, for the rule over
    /// the changed faces only (collapse_edge_after()). ring1_max is the max of tri_energy() over
    /// v1's ring, every face of which the collapse reshapes or removes; v2_only holds
    /// (fid, tri_energy) of the faces of v2's ring outside v1's ring, sorted by fid: the collapse
    /// leaves their slots and vertices alone, so one of them changes energy only through a front
    /// chord relabelled across it, and an unchanged one is in neither max. As in 3D.
    struct CollapseCells
    {
        double ring1_max = 0.;
        std::vector<std::pair<size_t, double>> v2_only;
    };
    mutable wmtk::threading::enumerable_thread_specific<CollapseCells> m_collapse_cells;
    /**
     * @brief The link of the collapsed edge, captured in collapse_before_vertex(): the only
     * vertices besides v2 whose offset membership a collapse can change.
     *
     * The chords that DIE are the ones carrying both endpoints -- in 2D only the edge itself, and
     * the faces (v1, v2, w) for w in the link take their two chords (v1, w) and (v2, w) down to
     * one -- so only v2 and those w can lose their last front chord; nothing but v2 can gain,
     * since edges only ever move from v1 to v2. Captured before the collapse because the edge is
     * gone by collapse_after_vertex(), which is where the refresh runs. As in 3D.
     */
    mutable wmtk::threading::enumerable_thread_specific<std::vector<size_t>> m_collapse_edge_link;
    /// The swap energy rule's before-half: the max of tri_energy() over the two faces the flip
    /// replaces, cached by swap_edge_before() and compared by swap_edge_after() and, early, by
    /// swap_quality_allowed(). The 3D twin is SwapEnergyBefore, which also carries the band cells
    /// beyond a 3-2 flip's faces; a 2D flip replaces two faces with two over the same quad, every
    /// quad edge keeping its outer face, so there is no "outside" to add.
    mutable wmtk::threading::enumerable_thread_specific<double> m_swap_energy_before;
    /// The live offset-boundary edges incident to vid, deduplicated. The 3D twin is
    /// offset_surface_faces_live_at().
    std::vector<Tuple> offset_surface_edges_live_at(size_t vid) const;
    /// Every live offset-boundary edge as a sorted vertex pair. The 3D twin is
    /// offset_surface_faces().
    std::vector<std::array<size_t, 2>> offset_surface_edges() const;
    /// Whether ANY live offset-boundary edge is incident to vid; reads only vid's own faces.
    bool vertex_has_live_offset_edge(size_t vid) const;
    /**
     * @brief Re-derive m_is_on_offset for one vertex from the face labels, exactly: a vertex is
     * on the offset boundary iff some incident edge has the band on one side and a non-complex
     * face on the other. Called from the hooks where an operation can change the answer -- the
     * collapse (v2 and the edge's link) and the split (the new vertex). Reads labels, never the
     * flag it is writing. Leaves m_vertex_attribute[vid].m_is_on_surface alone (the base's
     * union over every tracked surface). As in 3D.
     */
    void refresh_offset_membership(size_t vid);
    /// perform_sanity_checks: how many vertices carry m_is_on_offset without a live offset edge,
    /// and how many are the other way round. Whole-mesh; zero is the invariant.
    std::pair<size_t, size_t> offset_membership_mismatches() const;
    /// Log offset_membership_mismatches() and throw when it is not {0, 0}. perform_sanity_checks
    /// only.
    void check_offset_membership(const char* when) const;

    /**
     * @brief Per-vertex 0/1: has the offset curve folded back on itself at this vertex? Debug
     * frame diagnostic; see write_vtu(). Costs one pass over the live offset edges, no field
     * evaluation.
     *
     * A vertex on the offset curve carries two live offset edges. Measured through either
     * side, the angle between them is 180 degrees where the curve is straight and 360 where the
     * two edges lie on top of each other with that side pinched to nothing. Over
     * FOLDOVER_OUTER_ANGLE_DEG through EITHER side is the fold, and the vertex gets 1.
     *
     * Which side is pinched is deliberately not determined; see the 3D twin, where the measured
     * folds pinch the background rather than the band. Since the two sides sum to 360, the test
     * is simply that the unsigned angle is under 360 minus the threshold. Vertices without
     * exactly two live offset edges are left 0, as are degenerate edges: this is a diagnostic,
     * and a number it cannot measure is not a fold.
     *
     * The 3D twin is TopoOffsetTetMesh::offset_surface_foldover_labels(), which asks the same
     * question of an offset-surface EDGE's two faces and marks that edge's two endpoints.
     */
    std::vector<char> offset_surface_foldover_labels() const;
    void log_smooth_trace() const;


    /**
     * @brief Are the tracked region boundaries actually contained by anything?
     *
     * A class-0 edge is dispatched to an envelope by its boundary mask, the symmetric difference
     * of its two faces' tags. An edge whose faces carry the same tags has an empty difference, so
     * envelope_for_mask() gives it nullptr: tracked as a region boundary and held by nothing.
     * Construction cannot produce one, so a non-zero count here is a hole opened afterwards.
     * Called at construction.
     */
    void log_region_edge_mask_health(const std::string& when) const;

    /**
     * @brief Which tracked edges are outside their envelope, and by how much.
     *
     * The shared pass driver's sanity_checks() reports "Edge [a, b] is outside!" but not which
     * envelope refused it, and the answer forks the diagnosis: offset-class (mask 0) means the
     * final pass moved the offset boundary out of the tube holding it; region-class
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
     * still the input's own: construct_offset() replaces the tags of every face the band grows
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
     * included -- carry a containment requirement, and so does the offset boundary in the
     * frozen-front final pass: m_offset_envelope holds it where the loop left it (see
     * containment_for()).
     *
     * The envelope holds the other tag regions where they are, and the input complex too. That
     * half is exactly TriWild's input envelope: the complex may be split, collapsed and smoothed,
     * and this is what bounds how far the result may drift from the geometry as loaded. The 2D
     * twin of TopoOffsetTetMesh::surface_envelope_for_face().
     *
     * Null means "no containment requirement", which the base handles by skipping the check.
     */
    std::shared_ptr<SampleEnvelope> surface_envelope_for_edge(
        const std::array<size_t, 2>& vids) const override
    {
        // Boundary geometry first: a segment on any tag-region boundary may not
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
        // Both families compose: whatever holds this segment holds it at once. The offset side
        // holds only in the frozen-front final pass (see containment_for()); in the loop the
        // front is what the optimization moves, so it contributes nothing there.
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

    /// ... and it is not contained by the offset tube either: placing the front is what moves
    /// the offset boundary, so the smoother asks for the region tubes alone. The operations hold
    /// the offset boundary to its tube through surface_envelope_for_edge() in the final pass; see
    /// containment_for(). As in 3D.
    std::shared_ptr<SampleEnvelope> smoothing_containment_envelope(const size_t vid) const override
    {
        return containment_for(vertex_boundary_mask(vid), /*on_offset=*/false);
    }

    /**
     * @brief Placement of a front vertex: the shared smoother with the offset's options. See
     * FrontSmooth2d.cpp.
     */
    bool smooth_front_vertex(const Tuple& t);
    /**
     * @brief A repulsion vertex in the passes and rounds before the march (repulsion_smoothing()):
     * the shared smoother at the front's options, against the sum of the per-tri energy over its
     * ring -- w AMIPS, and O(v) once for every ring face that carries it (repulsion_energy(),
     * repulsion_cell_term()); under offset_front_smooth_veto, the max of the per-tri energy over
     * its ring may not rise. See FrontSmooth2d.cpp.
     */
    bool smooth_repulsion_vertex(const Tuple& t);
    /**
     * @brief Every vertex off the front (and off the plastic medium): the shared smoother with
     * TriWild's options (TriOptimizerMesh::smooth_after()), except that it minimises the per-cell
     * energy -- smoothing_extra_energy(), w AMIPS over the ring, the smoother adding no AMIPS
     * term of its own -- and the veto compares the max of tri_energy() over the ring, under the
     * same key (offset_smooth_veto) and on the same vertices as the engine's AMIPS veto it
     * replaces. See Optimize2d.cpp.
     */
    bool smooth_nonfront_vertex(const Tuple& t);
    /// w AMIPS over vid's one-ring, the per-cell energy's AMIPS part in the smoother's form
    /// (optimization::AMIPSEnergy2D, the engine's own quality), w = offset_amips_weight. The 3D
    /// twin is amips3_energy(), AMIPS^3 being the 3D engine's quality.
    std::shared_ptr<polysolve::nonlinear::Problem> amips_energy(size_t vid) const;
    /// ||grad F|| at front vertex vid along its move direction, F the objective
    /// smooth_front_vertex() minimises. +inf if unmeasurable.
    double front_vertex_normal_gradient(size_t vid) const;
    /// The line a front vertex is placed along: the field normal, or the boundary tangent where
    /// an input envelope holds it. See the definition.
    Vector2d front_vertex_move_direction(size_t vid) const;
    /// |cos| between front_vertex_move_direction() and the field normal: 1 means the convergence
    /// test's 1-D step is the step toward the level set, 0 means it measures a direction that
    /// cannot reduce the distance. Debug-frame diagnostic; see write_vtu().
    double front_move_alignment(size_t vid) const;
    /// The vertex measure over the one bar: |relative_residual(x)| / front_conv_frac(), the
    /// chord term's order-0 stencil at one end. Infinite when unmeasurable. As in 3D.
    double front_vertex_conv_ratio(size_t vid) const;
    /**
     * @brief THE definition of "placed" for a vertex on the offset boundary, applied to its
     * already-measured front_vertex_conv_ratio(): finite and within the one bar. Every caller that
     * asks "is the placement of this front vertex done" -- energy_criterion()'s vertex count and
     * the placed split of the chords over the bar, and the adaptive-smoothing stop -- goes through
     * here. Unmeasurable (a non-finite ratio) is NOT placed.
     */
    bool front_placed_by_ratio(const double ratio) const
    {
        return std::isfinite(ratio) && ratio <= 1.;
    }
    /**
     * @brief THE ONE CHORD FUNCTION: the mean over the chord's stencil of r^2, r =
     * relative_residual(q) / front_conv_frac() -- the squared chord measure, in units of the
     * tolerance, 1 at the bar. The 2D twin of TopoOffsetTetMesh::face_offset_term().
     *
     * Every reader of the chord measure goes through it: the logs print its root,
     * energy_criterion()'s exit, refinement and ring measures, the debug frames' front_err_ratio
     * and front_ring_ratio, and the per-cell energy tri_energy(), which adds it to the band face
     * of every live front chord. -1 when unmeasurable (a sample whose relative_residual() is not
     * finite, which the euclidean field never has), +inf when the bar is not positive.
     *
     * The (a, b) form reads the field of the chord (potential_for_edge()); the energy passes its
     * band face's field and its ends sorted, so a chord's term does not depend on which face or
     * which operation asks for it.
     */
    double edge_offset_term(size_t a, size_t b) const;
    double edge_offset_term(const OffsetPotential2D& pot, const Vector2d& pa, const Vector2d& pb)
        const;
    /// The field's outward unit direction at front vertex vid (zero where grad Phi vanishes).
    Vector2d front_vertex_normal(size_t vid) const;
    /// The objective of front vertex vid, as the smoother assembles it for a front vertex it
    /// places against the offset term (smoothing_extra_energy()): w AMIPS over the one-ring
    /// (amips_energy()), the rest-shape AMIPS of its plastic faces (rest_energy_for_vertex(),
    /// null without the plastic medium), and front_energy(). What
    /// front_vertex_normal_gradient() differentiates. As in 3D, where the AMIPS part is cubed.
    std::shared_ptr<polysolve::nonlinear::Problem> front_objective(size_t vid) const;
    /// Whether the smoother places vid against the offset term: a front vertex, outside the
    /// frozen-front final pass, that no input envelope also pins.
    bool vertex_carries_offset_term(const size_t vid) const
    {
        return !m_freeze_front && m_offset_potential && m_vertex_extra[vid].m_is_on_offset &&
               vertex_boundary_mask(vid) == 0;
    }
    /// The AMIPS weight the rest-shape term uses at vid: offset_amips_weight for a vertex placed
    /// against the offset term, the engine's w_amips factor otherwise. As in 3D.
    double smoother_amips_weight(const size_t vid) const
    {
        if (vertex_carries_offset_term(vid)) return m_offset_params.offset_amips_weight;
        return m_params.w_amips > 0 ? m_s_amips * m_params.w_amips : 1.0;
    }
    /// THE smoothing objective at vid, for every vertex the shared smoother places: the per-cell
    /// energy tri_energy() = w AMIPS + O summed over vid's ring, up to the terms that do not
    /// move with vid. w AMIPS over the ring (amips_energy()) for every vertex -- the smoother adds
    /// no AMIPS term of its own, every caller passing opts.w_amips 0 -- plus the offset terms of
    /// the front chords at vid when it is a front vertex the loop places (front_energy(); null in
    /// the final pass and for a front vertex an input envelope also pins), or, in the passes
    /// before the march, a repulsion vertex's term once per ring face that carries it
    /// (repulsion_energy(), repulsion_cell_term()). Under deform_others
    /// the rest-shape AMIPS of the deformable faces in the ring is added. As in 3D.
    std::shared_ptr<polysolve::nonlinear::Problem> smoothing_extra_energy(
        const size_t vid) const override
    {
        auto sum = std::make_shared<optimization::EnergySum>();
        sum->add_energy(amips_energy(vid));
        // Before the march (repulsion_smoothing()) there is no front and nothing is plastic.
        if (is_repulsion_vertex(vid)) {
            sum->add_energy(repulsion_energy(vid));
            return sum;
        }
        if (vertex_carries_offset_term(vid)) {
            sum->add_energy(front_energy(vid, potential_ptr_for(vid)));
        }
        if (const auto rest = rest_energy_for_vertex(vid)) sum->add_energy(rest);
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
    /// Under deform_others the same set as face_is_plastic(): every face outside the band.
    bool face_is_deformable(size_t fid) const;
    /// Plastic medium: under deform_others every background face -- ambient and the other objects
    /// alike -- is plastic, its rest shape re-stamped before every operation group, so smoothing
    /// resists only the increment since the group started and the medium flows instead of behaving
    /// as an elastic solid glued to the walls. The band (label 2) and the complex (label 1) are
    /// not plastic; element quality in the medium is the operation passes' job.
    bool m_plastic_active = false; ///< set in optimize_offset() when deform_others
    bool face_is_plastic(size_t fid) const
    {
        // Everything outside the band: ambient, the other objects and the input complex's
        // interior alike -- one material. The complex's boundary is what its tube holds.
        return m_plastic_active && m_face_extra[fid].label != 2;
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
    /// shared smoother weights its AMIPS term at vid (smoother_amips_weight()); null when the
    /// ring has none.
    std::shared_ptr<polysolve::nonlinear::Problem> rest_energy_for_vertex(size_t vid) const;
    /// The offset term for a front vertex, at offset_term_weight(): StencilEnergy2D over its
    /// incident live front chords, whose value is sum_e O(e), the terms the per-cell energy
    /// carries. Defined in FrontSmooth2d.cpp.
    std::shared_ptr<polysolve::nonlinear::Problem> front_energy(
        size_t vid,
        const std::shared_ptr<const OffsetPotential2D>& pot) const;

    /// Samples per front chord; see for_each_edge_sample().
    int stencil_order() const { return m_offset_params.stencil_order; }
    /// How many points for_each_edge_sample() visits at the configured order: 2 at order 0, and
    /// 2n + 1 with n = 2^(k-1) above it, i.e. 3, 5, 9, 17, ... Kept in step with
    /// for_each_edge_sample() by the unit test `stencil-order-point-counts-2d`. The 3D twin is
    /// stencil_points_per_face().
    int stencil_points_per_edge() const
    {
        const int k = m_offset_params.stencil_order;
        if (k < 0) return 0;
        if (k == 0) return 2;
        return 2 * (1 << (k - 1)) + 1;
    }

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

    /// A quantity sampled at the stencil points of a front chord, ends included
    /// (for_each_edge_sample()).
    struct EdgeSamples
    {
        double max = 0.;
        double sum = 0.;
        size_t n = 0;
    };

    /**
     * @brief The stencil_order stencil a chord is sampled on, handed to `visit` one point at a
     * time as (point, wa, wb) with the barycentric weights that built it. The 2D twin of
     * TopoOffsetTetMesh::for_each_face_sample(), the same rule one dimension down: order 0 is
     * the two ends alone; order k >= 1 is the vertices of the chord cut into n = 2^(k-1) equal
     * pieces plus the midpoint of each piece, so the counts are 2, 3, 5, 9, 17 for k = 0..4
     * (stencil_points_per_edge()). The ends are in it on purpose: the quantity sampled is a
     * distance to the level set, which at an end is that vertex's own placement error (see
     * edge_offset_term()).
     *
     * The weights are handed out for the front smoother: with the moving vertex x as p0, a
     * sample is wa*x + wb*p1, so it moves by wa per unit of x; StencilEnergy2D keeps the weights
     * fixed while the points slide with x (stencil_edge_at() in FrontSmooth2d.cpp).
     *
     * Takes positions rather than a Tuple: edge_offset_term() reads a chord's ends in sorted
     * order (see tri_energy()).
     */
    template <typename Visit>
    void for_each_edge_sample(const Vector2d& p0, const Vector2d& p1, Visit&& visit) const
    {
        const int k = m_offset_params.stencil_order;
        if (k < 0) return;

        const auto emit = [&](const double wa, const double wb) {
            visit(Vector2d(wa * p0 + wb * p1), wa, wb);
        };
        // Order 0 is the two ENDS alone, each end's own placement error.
        if (k == 0) {
            emit(1., 0.);
            emit(0., 1.);
            return;
        }
        // Order k >= 1: the vertices of the chord cut into n = 2^(k-1) pieces, then the midpoint
        // of each piece -- 2n + 1 points: 3, 5, 9, 17, ...
        const int n = 1 << (k - 1);
        const double dn = double(n);
        for (int i = n; i >= 0; --i) {
            emit(double(i) / dn, double(n - i) / dn);
        }
        const double d2n = 2. * dn;
        for (int i = n - 1; i >= 0; --i) {
            emit((2. * i + 1.) / d2n, (2. * (n - 1 - i) + 1.) / d2n);
        }
    }

    /// The same lattice over a chord the mesh carries. Visitor signature as above.
    template <typename Visit>
    void for_each_offset_edge_sample(const Tuple& e, Visit&& visit) const
    {
        for_each_edge_sample(
            m_vertex_attribute[e.vid(*this)].m_posf,
            m_vertex_attribute[e.switch_vertex(*this).vid(*this)].m_posf,
            std::forward<Visit>(visit));
    }

    /// The Phi residual at the `stencil_order` stencil's points of front chord `e`. Returns
    /// nothing for a chord with an unreachable end. The 3D twin is offset_face_samples().
    EdgeSamples offset_edge_samples(const Tuple& e) const;

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
     * is the edge-interior half and gates alongside the vertex half.
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
        /// Edge-interior samples measured into max_in_edge (not part of n_reachable).
        size_t n_edge_samples = 0;
        /// Band vertices the smoother would refuse to place, so their gradient is not part of
        /// the fixed point this measures.
        size_t n_skipped_inverted = 0, n_skipped_unrounded = 0;
    };
    /// @param include_edge_samples false skips the edge-interior half (the expensive one).
    /// Every convergence decision passes true.
    GradientSplit gradient_split(bool include_edge_samples = true) const;

    /**
     * @brief The front's measures against the one bar and the refinable chords. The 2D twin of
     * TopoOffsetTetMesh::EnergyCriterion, front chords in place of offset faces.
     */
    struct EnergyCriterion
    {
        double max_vertex = 0., max_edge = 0.; ///< ratios to the bar (1 = bar)
        /// Running sums of the SAME ratios, over the same measurable simplices the maxima are
        /// taken over, so avg_vertex() / avg_edge() below are the plain means of what max_vertex
        /// / max_edge report the largest of. Reported only; nothing tests them.
        double sum_vertex = 0., sum_edge = 0.;
        double bar = 1.;
        size_t n_vertices = 0, n_edges = 0, n_unmeasurable = 0;
        size_t worst_vid = static_cast<size_t>(-1);
        /// Reported only: the chords over the bar, split by whether both ends are PLACED
        /// (front_placed_by_ratio(), the one notion).
        size_t n_edges_over = 0, n_edges_over_placed = 0;
        double max_edge_placed = 0.;
        Vector2d worst_placed_mid = Vector2d::Zero();
        double tube = 0.;
        /// Chords over the bar that the refinement does not take: no chord target below the
        /// larger sizing scalar at their ends. Like every chord over the bar they block the exit.
        /// Two states share this count (see energy_criterion()): ends at the sizing floor, which
        /// nothing can refine, and a chord still at least twice the target length at its ends,
        /// which the split pass shortens. The first is n_corners_at_floor.
        size_t n_at_floor = 0;
        double max_edge_at_floor = 0.; ///< the worst of them, as a ratio to the bar
        Vector2d worst_at_floor_mid = Vector2d::Zero();
        double worst_at_floor_scalar = 0.; ///< the larger sizing scalar at the worst one's ends
        /// Of n_at_floor, the chords whose larger end scalar IS the floor: they cannot be
        /// refined, so a run keeping them cannot converge. The loop warns with them every turn
        /// they exist, and the verdict and the throw_on_nonconvergence message quote the same
        /// sentence, sizing_floor_fact(). Named as the 3D twin's.
        size_t n_corners_at_floor = 0;
        double max_edge_corners_at_floor = 0.; ///< the worst of them, as a ratio to the bar
        Vector2d worst_corners_at_floor_mid = Vector2d::Zero();
        /// The floor, max(min_sizing_scalar, min_edge_length / l), and which of the two it is.
        double floor_scalar = 0.;
        bool floor_from_min_edge_length = false;
        size_t n_unplaced = 0; ///< measurable front vertices that front_placed_by_ratio() refuses
        /// A front chord over the bar: a, b its ends; `measure` its RMS error as a LENGTH (the
        /// ratio times the tube), kept as the 3D twin keeps it; len the chord's length.
        struct Refinable
        {
            size_t a, b;
            double measure, len;
        };
        std::vector<Refinable> refinable;
        /// THE RING MEASURE, filled only under front_measure "vertex_ring": the 3D twin's, with
        /// the front chords in place of the offset faces. At a front vertex v, over the front
        /// chords incident to v that the chord loop measured (both ends front vertices) -- two at
        /// a vertex inside the front, one at the end of an open one:
        ///
        ///     r_v = sqrt( (1/n_v) sum_e O(e) ),   O(e) = edge_offset_term()
        ///
        /// every chord weighted equally: the front smoother's own offset term at v
        /// (StencilEnergy2D at offset_term_weight() is sum_e O(e) = n_v r_v^2), the same chords'
        /// terms the per-cell energy tri_energy() carries. A vertex with any unmeasurable incident
        /// chord has no ring measure (n_rings_unmeasurable; the chord itself is already in
        /// n_unmeasurable).
        bool ring_exit = false; ///< front_measure "vertex_ring"
        static const char* ring_name() { return "ring measure"; }
        double max_ring = 0., sum_ring = 0.; ///< ratios to the bar (1 = bar)
        size_t n_rings = 0, n_rings_unmeasurable = 0;
        size_t worst_ring_vid = static_cast<size_t>(-1);
        size_t n_rings_over = 0; ///< vertices whose ring measure is over the bar
        /// The ring-mode refinement: every vertex over the bar whose sizing scalar the halving can
        /// still lower, handed to refine_front_by_halving() as the vertex alone.
        std::vector<size_t> refinable_vertices;
        /// Vertices over the bar whose sizing scalar is already at the floor: nothing can refine
        /// them, so they block the exit for good. sizing_floor_fact() names them in ring mode.
        size_t n_rings_at_floor = 0;
        double max_ring_at_floor = 0.; ///< the worst of them, as a ratio to the bar
        Vector2d worst_ring_at_floor_pos = Vector2d::Zero();
        bool rings_ok() const { return max_ring <= bar; }
        double avg_ring() const { return n_rings ? sum_ring / double(n_rings) : 0.; }
        /// Every front vertex placed: the VERTEX measure, a DIAGNOSTIC only. Counted through
        /// front_placed_by_ratio() rather than re-derived from max_vertex, so the reported count
        /// and the per-vertex notion cannot drift apart. Nothing in the exit or the verdict tests
        /// it since 2026-09-25; see converged().
        bool vertices_ok() const { return n_unplaced == 0; }
        bool edges_ok() const { return max_edge <= bar; }
        /// THE exit test, and the front half of the run's verdict: every front chord's measure
        /// within the bar AND nothing unmeasurable -- the chord measure is the root of
        /// edge_offset_term(), the RMS relative error over the chord's stencil, which samples its
        /// two ends as well as its interior. Under front_measure "vertex_ring" the ring measure
        /// takes the chord measure's place: every front vertex's ring measure within the bar AND
        /// nothing unmeasurable, the chord measure then reported only. As in 3D.
        bool converged() const
        {
            return (ring_exit ? rings_ok() : edges_ok()) && n_unmeasurable == 0;
        }
        /// The n_at_floor chords as one sentence, for the turn's line (a warning when some have
        /// their ends at the sizing floor), the verdict and the throw_on_nonconvergence message
        /// alike, so all three state the same fact. Empty when n_at_floor is 0. Under
        /// front_measure "vertex_ring" the same three places get the n_rings_at_floor vertices
        /// instead, empty when there are none.
        std::string sizing_floor_fact() const;
        double ratio() const { return bar > 0. ? std::max(max_vertex, max_edge) / bar : 0.; }
        /// Means over the measurable front vertices / front chords; 0 when there are none.
        double avg_vertex() const { return n_vertices ? sum_vertex / double(n_vertices) : 0.; }
        double avg_edge() const { return n_edges ? sum_edge / double(n_edges) : 0.; }
    };
    EnergyCriterion energy_criterion();
    /// The edge length that would bring a front chord's error under the tube: 3/4 L
    /// (tube / sag)^(1/p) capped at L/2, with the exponent p measured from how the level set
    /// turns across the chord (2 where it is smooth, 1 where the chord straddles a kink).
    /// Reached from ONE place: the refinable / at-floor test in energy_criterion(). Same formula
    /// as 3D.
    double front_chord_target(size_t va, size_t vb, double len, double sag, double tube) const;

    /// THE refinement: halve the sizing scalar at the ends of every refinable chord, once per
    /// vertex per call, floored at max(min_sizing_scalar, min_edge_length / l), then graded
    /// outward. Returns the number of vertices lowered.
    size_t refine_front_by_halving(const std::vector<EnergyCriterion::Refinable>& edges);
    /// The same halving at the listed vertices themselves: each lowered once per call, floored,
    /// then graded. The edge form above is this on its chords' ends, in the order given;
    /// front_measure "vertex_ring" calls it directly with the vertices whose ring measure is over
    /// the bar. The 3D twin has the same pair.
    size_t refine_front_by_halving(const std::vector<size_t>& vertices);

    /// Spread the refinement just made at `seeds` to the vertices around them, the way
    /// sizing_gradation_mode says: "ring" is the base gradation_smooth_sizing(grade, seeds),
    /// "distance" is grade_sizing_by_distance(seeds) and ignores `grade`. Every place the
    /// offset lowers the field goes through here.
    void grade_sizing(double grade, const std::vector<size_t>& seeds);
    /// TetWild's gradation, ported from tetwild::TetWild::adjust_sizing_field: a breadth-first
    /// walk out of the seeds over mesh neighbours; every vertex reached within R = 1.8 l of its
    /// nearest seed has its scalar multiplied by 0.5 + 0.5 dist / R, the walk stops at vertices
    /// farther than R, and the result is floored at the sizing floor. The seeds themselves keep
    /// the scalar the caller gave them. Returns the number of vertices lowered.
    size_t grade_sizing_by_distance(const std::vector<size_t>& seeds);

    /// What one interleaved smoothing pass achieved, measured after it against the positions
    /// before it. See smooth_group_to_convergence() and the adaptive_smoothing keys.
    struct SmoothingProgress
    {
        /// Max front_vertex_conv_ratio over the measurable front vertices (1 = the bar).
        double front_max_ratio = 0.;
        size_t front_worst_vid = static_cast<size_t>(-1);
        size_t n_front = 0; ///< front vertices the max was taken over
        size_t n_front_unmeasurable = 0; ///< ratio not finite: left out of the max
        double front_max_step = 0.; ///< max front step in the pass, in tube half-widths
        /// Max over non-front vertices of the step in the pass divided by the vertex's target
        /// edge length s_v * l.
        double background_max_step = 0.;
        size_t background_worst_vid = static_cast<size_t>(-1);
        size_t n_background = 0;
    };
    /// Measure a pass: `before` holds every live vertex's position before it, indexed by vid.
    SmoothingProgress smoothing_progress(const std::vector<Vector2d>& before);
    /// The interleaved smoothing of one operation group under adaptive_smoothing: one pass at a
    /// time through local_operations({0,0,0,1}), each followed by smoothing_progress(), until
    /// the front has converged (max ratio <= 1) or stalled (max ratio fell by less than
    /// adaptive_smoothing_stall_rel) AND the background has settled (max step <=
    /// adaptive_smoothing_step_rel x its target edge), or adaptive_smoothing_max_passes.
    void smooth_group_to_convergence(const char* group_name);
    /// The energy criterion as measured when the loop converged; the final pass runs after
    /// it and the verdict must not be re-measured on that mesh.
    std::optional<EnergyCriterion> m_energy_verdict;


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

    /**
     * @brief TriWild's stall-driven sizing refinement, verbatim.
     *
     * Structurally TriWildMesh::refine_sizing_around_worst, down to the shared helpers in
     * wmtk/utils/SizingField.hpp and every stuck_refine_* parameter: rank faces by AMIPS,
     * force-split the worst ones' longest edges, grow the region by rings, lower the
     * per-vertex sizing scalar, grade it outward.
     *
     * Final pass only: mesh_improvement() is its one caller, and the loop only runs that as the
     * frozen-front final pass.
     */
    size_t refine_sizing_around_worst(double max_metric) override;

    /**
     * @brief Why the final pass is stuck: a census of the faces stuck-refine is about to chase.
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
     * stopping the mesh from fixing them", the question that matters when the final pass refines
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
     * @brief The energy rule's EARLY half, on the engine's lower bound.
     *
     * The engine scores each reshaped face before the collapse exists and hands its AMIPS in as
     * `q`. tri_energy() of that face is weighted_amips(q) plus a non-negative chord term, so q
     * alone above the before-maximum (collapse_before_vertex()) already decides: the rule that
     * collapse_edge_after() applies on the real faces would refuse too. Refusing here costs
     * nothing; refusing there costs the collapse and its rollback. Same rule, same counter,
     * applied as soon as it can be. The engine's own `ring_max` (AMIPS over v1's ring) is not
     * read; TriWild's exemption for an unrounded v1 is kept. Off with offset_collapse_veto. As in
     * 3D.
     */
    bool collapse_quality_allowed(size_t v1, size_t /*v2*/, double q, double /*ring_max*/)
        const override
    {
        if (!m_offset_params.offset_collapse_veto || !m_vertex_attribute.at(v1).m_is_rounded) {
            return true;
        }
        if (weighted_amips(q) <= m_collapse_energy_before.local()) return true;
        ++iter_cnt_collapse_energy_reject;
        return false;
    }

    /**
     * @brief The energy rule's EARLY half for a swap, on the engine's lower bound.
     *
     * `after` is the largest AMIPS over the two new faces. tri_energy() of a face is
     * weighted_amips() of its AMIPS plus a non-negative chord term, so `after` not strictly below
     * the before-maximum (swap_edge_before()) already decides what swap_edge_after() would decide
     * on the real faces: refused. The engine's `before` (stored AMIPS of the old faces) is not
     * read. Off with offset_swap_veto. A 2D swap never flips the offset boundary (the engine
     * refuses every tracked edge), so `is_surface_flip` is always false. As in 3D.
     */
    bool swap_quality_allowed(double after, double /*before*/, bool) const override
    {
        if (!m_offset_params.offset_swap_veto) return true;
        if (weighted_amips(after) < m_swap_energy_before.local()) return true;
        ++iter_cnt_swap_energy_reject;
        return false;
    }

    mutable std::atomic<size_t> m_deg_split_created{0};
    size_t m_deg_prev_split_created = 0;

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
    /// after construction, at each collapse pass. Reports the count and the worst few.
    void needle_scan(const char* when) const;
    /// Diagnostic only: the base offers no per-iteration hook except this one, so the needle
    /// population scan rides on it. Calls nothing else -- the base default is empty.
    void collapse_pass_begin() override;

    /// Quantised centroids of the MAX_ENERGY faces at the previous stuck-refine, for the overlap
    /// line above. Diagnostic only; nothing reads it but log_stuck_refine_census().
    std::set<std::pair<long, long>> m_stuck_prev_cells;
    size_t m_stuck_calls = 0;

    /// TriWild's bare collapse passes are off for the offset, as TetWild's are in 3D: with no
    /// length gate the quality test alone demolishes the band, and the sizing field cannot refuse
    /// a collapse.
    bool optimization_bare_coarsen_passes() const override { return false; }

    /// Max of the two normalized criteria on this face: AMIPS over stop_energy, and on each live
    /// chord it carries the root of edge_offset_term() (the chord's RMS relative error over its
    /// stencil in units of the bar; +inf if unmeasurable). > 1 means it fails at least one. The
    /// coarsen-mode collapse accept reads it. As in 3D.
    double face_criterion_rel(const size_t fid) const;
    /**
     * @brief Put the optimization's frames on the run's single debug timeline (see
     * write_debug_frame()), labelled "r<turn><tag><pass>_<op>" / "r<turn><tag>_end", tag S in
     * the loop and F in the final pass; led by m_frame_prefix (i during init_optimize). Same
     * scheme as 3D.
     */
    void write_smoothing_debug_output(const std::string& path) const override
    {
        const char ph = m_freeze_front ? 'F' : 'S'; // the final pass, or the loop
        if (m_round != m_debug_last_round || ph != m_debug_last_tag ||
            m_frame_prefix != m_debug_last_prefix) {
            m_debug_last_round = m_round;
            m_debug_last_tag = ph;
            m_debug_last_prefix = m_frame_prefix;
            m_debug_pass = 0;
        }
        std::string label = path;
        // The prefix leads, since the loop after the init_optimize loop restarts at turn 1.
        const std::string& g = m_frame_prefix;
        if (path.rfind("debug_", 0) == 0) {
            label = fmt::format(
                "{}r{}{}{}{}",
                g,
                m_round,
                ph,
                ++m_debug_pass,
                m_debug_pass_name.empty() ? std::string() : "_" + m_debug_pass_name);
        } else if (path.rfind("end_", 0) == 0) {
            label = fmt::format("{}r{}{}_end", g, m_round, ph);
        }
        const_cast<TopoOffsetTriMesh*>(this)->write_debug_frame(label);
    }
    /**
     * @brief Called by the engine at every smoothing pass boundary (and only there in this
     * component's loop): the per-pass smoothing accounting the 3D engine prints through its
     * log_smoothing_pass_accounting() hook, which the 2D engine does not have, then the debug
     * frame the base default writes. Writes the log and the frame, never the mesh.
     */
    void optimization_debug_checkpoint() override;
    /// The per-pass smoothing lines: the Newton counters, the front veto, the gradient bins and,
    /// under DEBUG_crossings, the crossings. The 3D twin overrides the engine hook of this name.
    void log_smoothing_pass_accounting();
    /// One line of <output>_frames.txt; truncates the file on the first frame.
    void append_frame_label(size_t idx, const std::string& label) const;
    /**
     * @brief One frame of the run's single debug timeline: <output>_NNNNN.vtu with the next
     * sequence number, and one "NNNNN<tab>label" line in <output>_frames.txt. Every debug
     * frame the run writes -- the input as loaded, the construction stages, and the
     * optimization's own frames through write_smoothing_debug_output() -- goes through
     * this sequence, so the numbers are consecutive and the .txt says what each
     * one is. The only debug files outside it are the ones that are not this mesh:
     * <output>_input_complex.vtu and the phi grid.
     */
    void write_debug_frame(const std::string& label);

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

    /// Construction, start to finish, on the input mesh as given: the simplicial embedding,
    /// repulsion_smoothing() when asked for, marching_tris() and the offset tagging. The
    /// optimization is optimize_offset(), which the driver calls afterwards.
    void construct_offset(const std::filesystem::path& output_file);

    /// Marching triangles: every edge with one endpoint in the input complex (label 1/2) and the
    /// other in the background (label 0) is split -- where d(x) = target_distance along the edge,
    /// or under construction_mode's fallback at half the maximum marchable distance or at the
    /// midpoint (see edge_split_sphere_trace()) -- and afterwards every background triangle
    /// still touching a complex frontier vertex (the split-off halves) becomes the band
    /// (label 2).
    void marching_tris();

    /// repulsion_smoothing_passes and repulsion_rounds (see the spec): between the simplicial
    /// embedding and marching_tris(), smoothing passes and then rounds of the loop's operations
    /// that push the outer ends of the marched edges out to 2 x target_distance, stopping once
    /// every outer end is beyond target_distance + front_conv. As in 3D.
    void repulsion_smoothing();
    /// Every construction label (vertex, edge, face) back to 0, then label_input_complex(): the
    /// input complex from the region tags, which the operations carry.
    void relabel_input_complex();

    /// Label connected simplicial complex components (simplices labelled 1 or 2) and return
    /// their number. The driver compares the count before and after construction. As in 3D.
    size_t flood_fill();
    /// All one-ring vertices through input / offset edges (labelled 1 or 2).
    std::vector<size_t> connected_components_helper(const size_t v_id) const
    {
        std::vector<size_t> onering_v_ids = get_one_ring_vids_for_vertex_duplicate(v_id);
        wmtk::vector_unique(onering_v_ids);
        std::vector<size_t> ret_v_ids;
        for (const size_t other_v_id : onering_v_ids) {
            if (other_v_id == v_id) continue;
            const size_t e_id = std::get<1>(tuple_from_edge({{v_id, other_v_id}}));
            if (m_edge_extra[e_id].label != 0) { // edge labelled 1 or 2
                ret_v_ids.push_back(other_v_id);
            }
        }
        return ret_v_ids;
    }


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
};


} // namespace wmtk::components::topological_offset
