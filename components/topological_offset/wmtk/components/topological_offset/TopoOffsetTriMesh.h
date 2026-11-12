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
#include <wmtk/MeshData.hpp>
#include <wmtk/envelope/CompositeEnvelopes.hpp>
#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include "ExactBandIntegral2D.hpp"
#include "OffsetPotential.hpp"
#include "Parameters.h"
#include "SimplicialComplexBVH.hpp"

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
    /// The construction label: 1 on the input complex, 2 on a band triangle off it, 0 elsewhere.
    /// The construction writes it; the optimization keeps it at every split's new vertex and every
    /// collapse's survivor and link (TopoOffsetTriMesh::carried_vertex_label()), from what 2D's
    /// operations carry. 3D keeps its labels by its carried edge labels (close_labels()); 2D's
    /// edge label (EdgeExtra2d::label) is not carried, and is read only before the optimization.
    int label = 0;
    /// The entry of Parameters::offsets whose input complex this vertex is on (label 1), set by
    /// label_input_complex(); -1 on every other vertex, a split's new vertex included. As
    /// VertexExtra::offset in 3D.
    int offset = -1;
    /// The labels of the input points on this vertex, bit j for the base's m_point_names[j]: set
    /// at load (init_from_image()) and carried by the construction, which only splits, a new
    /// vertex starting from none (marching_split_edge_before(), split_face_before()), until
    /// hold_curves_and_points() anchors the points and clears it: from there on the anchors carry
    /// them, through the operations, which do not carry this. As VertexExtra::point_inputs in 3D.
    /// A mesh not held (an empty input complex) writes its point block from this
    /// (collect_loaded_curves_and_points()).
    InputSet point_inputs = 0;
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

    /// The input-complex vertex this offset vertex was split off from by marching_tris(): the
    /// marched edge's endpoint on the complex. -1 for every vertex the march did not make. Written
    /// into the output .msh by save_offset_correspondence; not maintained by the optimization.
    /// The 3D twin is VertexExtra::m_corr_input_vid.
    int64_t m_corr_input_vid = -1;
    /// m_corr_input_vid came with the input, from an earlier offset (topological_offset.cpp), not
    /// from this run's march. split_edge_before() throws rather than cut this vertex's edge to the
    /// vertex it names. As VertexExtra::m_corr_carried in 3D.
    bool m_corr_carried = false;

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
    /// The construction label, as VertexExtra2d::label; not carried by the optimization's
    /// operations, so read only before it (and by the label clean-up's flips, which rewrite it).
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
    /// face set whose boundary it is, so it is selectable only through this tag: a selection (offsets)
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
    /// The distance from p to entry k of Parameters::offsets alone, as m_input_complex_bvh's
    /// dist() for the whole complex: 0 inside the entry's faces, the distance to its primitives
    /// elsewhere (infinite for an entry that selects nothing). The march traces each entry's edges
    /// on it. One entry: m_input_complex_bvh itself. As TopoOffsetTetMesh::offset_distance().
    double offset_distance(size_t k, const Vector2d& p) const;
    /// offset_distance()'s engine for each entry, built by init_input_complex_bvh() only when
    /// there are several: a BVH over the entry's rows of m_phi_* (its faces, segments and points);
    /// null for an entry that selects nothing. 3D's OffsetQuery holds an envelope and a winding
    /// number hierarchy for the inside test; 2D's BVH answers both (0 inside a face of the
    /// complex), as m_input_complex_bvh does for the whole complex.
    std::vector<std::shared_ptr<SimplicialComplexBVH>> m_offset_queries;
    /// Per entry of Parameters::offsets: its complex has no faces (one entry: the whole complex).
    /// init_input_complex_bvh(). As in 3D, where it means no triangles.
    std::vector<bool> m_offset_curves;
    bool offset_curves(const size_t k) const
    {
        return k < m_offset_curves.size() ? m_offset_curves[k] : m_phi_F.rows() == 0;
    }
    /// The entry of Parameters::offsets a complex vertex (label 1) is on: 0 when there is one, so
    /// that a complex labelled by hand (the unit tests) is that entry's. As in 3D.
    size_t offset_of(const size_t vid) const
    {
        return m_offset_params.offsets.size() <= 1 ? 0 : size_t(m_vertex_extra[vid].offset);
    }
    /// Entry k's target_distance; the run's when there is one entry, as offset_of(). As in 3D.
    double offset_target(const size_t k) const
    {
        return m_offset_params.offsets.size() <= 1 ? m_offset_params.target_distance
                                                   : m_offset_params.offsets[k].target_distance;
    }

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
    /// Several entries of Parameters::offsets: the field of the place's piece (place_piece()),
    /// each piece's at its own distance; one entry: the band region's, as before. As in 3D.
    const OffsetPotential2D& potential_for(const size_t vid) const
    {
        if (m_offset_params.offsets.size() > 1) return potential_for_region(place_piece(&vid, 1));
        return potential_for_region(vertex_region(vid));
    }
    /// The same selection as potential_for(), as the pointer the energies take a share of. Null
    /// only when m_offset_potential is, which the front placement paths test for.
    std::shared_ptr<const OffsetPotential2D> potential_ptr_for(const size_t vid) const
    {
        const int r =
            m_offset_params.offsets.size() > 1 ? place_piece(&vid, 1) : vertex_region(vid);
        return (r >= 0 && size_t(r) < m_region_potentials.size()) ? m_region_potentials[size_t(r)]
                                                                  : m_offset_potential;
    }
    const OffsetPotential2D& potential_for_edge(const size_t va, const size_t vb) const
    {
        if (m_offset_params.offsets.size() > 1) {
            const std::array<size_t, 2> e{{va, vb}};
            return potential_for_region(place_piece(e.data(), 2));
        }
        return potential_for_region(edge_region(va, vb));
    }
    const OffsetPotential2D& potential_for_face(const size_t fid) const
    {
        if (m_offset_params.offsets.size() > 1) {
            const std::array<size_t, 3> vs = oriented_tri_vids(fid);
            return potential_for_region(place_piece(vs.data(), 3));
        }
        return potential_for_region(fid < m_face_region.size() ? m_face_region[fid] : -1);
    }
    /**
     * @brief Several entries of Parameters::offsets, each piece its entry's target distance: the
     * piece of the input complex a place reads its distance, its bar and its exact integral from.
     * The 2D twin of TopoOffsetTetMesh::place_piece(). A place is given by its vertices (a vertex,
     * an edge, a face). The piece of the band they are in (m_vertex_region); where two pieces
     * meet -- vertices of two, or a vertex on band faces of two (-2) -- the one of them with the
     * smaller distance; off every band, or on vertices not assigned yet (an operation's new
     * vertex, until assign_band_regions() runs again), the nearest piece (nearest_piece()). -1
     * with one entry, or before the per-piece fields exist: the whole complex's field and
     * integral and the run's distance.
     */
    int place_piece(const size_t* vids, size_t n) const;
    /// The piece nearest to p: that of the input-complex BVH's nearest feature (m_phi_vert_region,
    /// m_phi_seg_region), as assign_band_regions() reads a seed's piece; -1 for none. 3D asks its
    /// per-piece envelopes (m_region_envelopes), which 2D does not build.
    int nearest_piece(const Vector2d& p) const;
    /// Per piece of the input complex (m_n_regions): the entry of Parameters::offsets it belongs
    /// to, from its vertices' VertexExtra2d::offset (init_input_complex_bvh()). All 0 for one
    /// entry. As in 3D.
    std::vector<int> m_region_offset;
    /// Piece r's target distance: its entry's (m_region_offset), its growth_mode stage's while a
    /// stage runs (m_offset_levels). The run's (the smallest entry's) for r < 0 and with one entry.
    /// As in 3D.
    double region_delta(const int r) const
    {
        if (m_offset_params.offsets.size() <= 1 || r < 0 || size_t(r) >= m_region_offset.size())
            return m_offset_params.target_distance;
        const size_t k = size_t(m_region_offset[size_t(r)]);
        return m_offset_levels.empty() ? m_offset_params.offsets[k].target_distance
                                       : m_offset_levels[k];
    }
    /// Several entries, growth_mode: each entry's target in the stage running (region_delta());
    /// empty outside the stages. One entry: the stages set target_distance itself. As in 3D.
    std::vector<double> m_offset_levels;
    /// Several entries, growth_mode: each entry's d_0, the distance its march traced to; its
    /// target_distance where it traced to that, or could not trace (marching_tris()). As in 3D.
    std::vector<double> m_growth_starts;
    /// The target distance at a place (place_piece()). As in 3D.
    double place_delta(const size_t* vids, const size_t n) const
    {
        return region_delta(place_piece(vids, n));
    }
    double vertex_delta(const size_t vid) const { return place_delta(&vid, 1); }
    /// front_conv at a place of piece r: front_conv_rel of its entry's target_distance, each piece
    /// its own (Parameters::init() refuses an absolute front_conv for entries of different
    /// distances), fixed through growth_mode's stages as front_conv is for one entry; front_conv
    /// itself with one entry. As in 3D.
    double front_conv_of(const int r) const
    {
        if (m_offset_params.offsets.size() <= 1 || r < 0 || size_t(r) >= m_region_offset.size())
            return m_offset_params.front_conv;
        return m_offset_params.front_conv_rel *
               m_offset_params.offsets[size_t(m_region_offset[size_t(r)])].target_distance;
    }
    /// The bar as a fraction of piece r's current target (front_conv_frac() with one entry): the
    /// unit of a relative residual read on piece r's field. As in 3D.
    double front_conv_frac_of(const int r) const
    {
        if (m_offset_params.offsets.size() <= 1 || r < 0 || size_t(r) >= m_region_offset.size())
            return m_offset_params.front_conv_frac();
        return front_conv_of(r) / std::max(region_delta(r), 1e-16);
    }
    /// The factor that puts a relative residual of chord e's piece in the run's units, |d - delta|
    /// for one entry: delta itself with one entry (`delta`, the run's), front_conv /
    /// front_conv_frac_of(the chord's piece) with several, so that a run-wide measure in lengths
    /// (edge_stencil_max_sq()) compared against front_conv^2 compares each chord's against its
    /// own piece's bar. 3D's run_units_scale() on a face.
    double run_units_scale(const std::array<size_t, 2>& e, const double delta) const
    {
        if (m_offset_params.offsets.size() <= 1) return delta;
        return m_offset_params.front_conv / front_conv_frac_of(place_piece(e.data(), 2));
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
    std::map<int64_t, std::shared_ptr<Envelope>> m_tag_envelopes;

    /// Input tag id -> bit position in VertexExtra2d::m_boundary_mask. Assigned in
    /// init_from_image() once the tag maps are complete; at most 64 input tags.
    std::map<int64_t, int> m_tag_bit;

    /// Memoized IntersectionEnvelope per multi-bit mask. Lazily built under the mutex because
    /// the queries that need them run concurrently under kPartition.
    mutable std::map<uint64_t, std::shared_ptr<Envelope>> m_isect_cache;
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
    mutable std::map<uint64_t, std::shared_ptr<Envelope>> m_offset_isect_cache;

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
    std::shared_ptr<Envelope> containment_for(uint64_t region_mask, bool on_offset) const;

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
     * stands when called: at load (PerTag, before the complex is labelled) and when deform_others
     * switches the setup at construction; not at the final pass, which holds the region
     * boundaries where the load did, as 3D's. The offset tube is separate and unchanged.
     */
    enum class EnvelopeSetup { PerTag, WallComplex };
    EnvelopeSetup envelope_setup() const
    {
        // deform_others was removed on the 3D-only branch spenbert/offsets-cleanup (2026-10-13);
        // its default, false, is hard-wired here until the 2D port.
        return /* deform_others */ false ? EnvelopeSetup::WallComplex : EnvelopeSetup::PerTag;
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
    /// deform_others switches it at construction. The tracked-edge flags are left alone: they are
    /// the topology the operations maintain. `when` labels the log line.
    void build_boundary_envelopes(const char* when, EnvelopeSetup setup);

    /**
     * @brief The tube the offset boundary may not leave during the frozen-front final pass, of
     * half-width offset_envelope, built from the boundary as the loop left it just before that
     * pass (containment_for() holds the final pass's operations to it). Null until then: the
     * loop holds the offset boundary to no envelope. Unlike m_tag_envelopes, which must never be
     * rebuilt. As in 3D.
     */
    std::shared_ptr<Envelope> m_offset_envelope;

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
    /// init_optimize loop (and a growth_mode stage before the last): it returns on convergence
    /// without the frozen-front final pass and without deciding the verdict. `refine` false skips
    /// the halving. `label` names the loop in its log. `max_turns` > 0 caps the turns in place of
    /// max_rounds (growth_mode "smart"'s attempts). `turned_free`: a converged turn with any
    /// offset edge turned from the field (turned_edge_census()) ends the loop as not converged,
    /// before the verdict and the final pass. Returns whether the loop converged (free of turned
    /// edges when asked) within its turns. As in 3D.
    bool optimize_offset_loop(
        bool final_stage = true,
        bool refine = true,
        const std::string& label = std::string(),
        int max_turns = 0,
        bool turned_free = false);
    /**
     * @brief The loop's local_operations(ops): the 2D twin of TopoOffsetTetMesh::loop_group(),
     * the same passes in the same order. Under EXPERIMENTAL_front_pocket_split, the group's
     * operation passes in one call, then EXPERIMENTAL_label_cleanup (with the held objects'
     * check), then each of its ops[3] smoothing passes in a call of its own, front_pocket_split()
     * right before each: the engine's group runs its operation pass straight into its smoothing
     * passes, so it offers no point before a smoothing pass. Off: local_operations(ops), and no
     * clean-up, which the parameters refuse without the pocket splits (Parameters). `where` names
     * the group in the [label cleanup] and [pocket split] lines.
     */
    void loop_group(const std::array<int, 4>& ops, const std::string& where);
    /**
     * @brief EXPERIMENTAL_front_pocket_split: the 2D twin of
     * TopoOffsetTetMesh::front_pocket_split(), one dimension down. Invariant: no cell is PINNED
     * when a smoothing pass of the loop starts. A pinned cell is a triangle outside the input
     * complex, band or not, whose three corners are all front vertices
     * (VertexExtra2d::m_is_on_offset).
     *
     * Why, as in 3D: only smoothing moves vertices, and no vertex joins or leaves the front during
     * a smoothing pass, so a triangle can only be flattened during smoothing, and only from one
     * that is pinned when the pass starts. The offset term pulls a pinned triangle's three corners
     * onto the level set, which then fixes its shape -- the chord of a curve and a third point on
     * it, a sliver -- and where the level set curves the wrong way flattens it to zero area, a
     * fold of the front; a triangle with a corner off the front has a vertex the AMIPS term alone
     * moves, which keeps it shaped.
     *
     * How. Each pinned triangle has its longest edge that is not a front edge
     * (edge_is_offset_surface_live() false) split by the loop's own split, the next longest when
     * that is refused. The midpoint is off the front and a corner of both children, so such a
     * split makes no pinned cell and removes one. A pinned triangle with all three edges on the
     * front, a one-triangle component of its label, has no edge to split and is counted only:
     * 3D's cell with all four faces on the offset surface. 3D's cell with all six edges but not
     * all four faces on it, split on a face's edge, has no twin: a triangle's faces are its
     * edges. EXPERIMENTAL_front_spanned_split (see the spec): every SPANNED edge -- both ends
     * front vertices, not a front edge, no triangle at it in the input complex -- is split first,
     * at its midpoint, off the front; the spanned triangle is the pinned one, and 3D's spanned
     * face is 2D's spanned edge (codimension 1, every corner on the front, not on the front
     * itself). Rescanned until nothing is left to split, at most kPocketMaxRounds rounds. One
     * [pocket split] line, `where` naming the pass. Before all of it, the chords the last settled
     * turn listed under EXPERIMENTAL_settle_mark "vertex_split" (m_settle_split_edges) are split,
     * with one [settle split] line, as in 3D. A no-op with the key off and in the final pass
     * (m_freeze_front; 3D: a TetWild pass).
     */
    void front_pocket_split(const std::string& where);
    /// 3D's cap on front_pocket_split()'s rescans (TopoOffsetTetMesh::kPocketMaxRounds), the
    /// same 10. In 2D a committed split makes no pinned cell and no spanned edge (its midpoint is
    /// off the front) and removes at least one, so the rescans end on their own; the cap binds
    /// only where the front flags are out of step with the labels (check_offset_membership()).
    static constexpr int kPocketMaxRounds = 10;
    /// EXPERIMENTAL_settle_mark "vertex_split": the front chords the last settled turn chose to
    /// split (settle_refinement_turn()); front_pocket_split() splits them and clears the list. As
    /// in 3D, whose list holds front edges of its faces.
    std::vector<std::array<size_t, 2>> m_settle_split_edges;
    /**
     * @brief EXPERIMENTAL_label_cleanup (see the spec). The 2D twin of
     * TopoOffsetTetMesh::label_cleanup(), one dimension down: a cell is a triangle, its faces
     * are edges. The one exception to labels being fixed after construction: a triangle on the
     * band / non-band interface whose own geometry puts it on the other side of the level set
     * takes the other label, where the flip keeps the topology of both labels and of every held
     * object. Invariant: the number of connected components (edge adjacency) of the band and of
     * the non-band region is the same before and after; a change throws.
     *
     * Serial passes in fid order over the triangles with a front edge, reading the current
     * labels, until a pass flips nothing; positions do not move, and a triangle's side test
     * reads its own corners alone, so a triangle flips at most once and the passes end. After
     * each flip every value derived from labels at the triangle's closure is re-derived (the
     * closed labels, the front class of its edges, the front flags of its corners); after the
     * passes the band-region map. One [label cleanup] line. Called by loop_group() after the
     * operation passes; a no-op with the key "off". "energy_decrease" reads the "surface" rule's
     * face term, which 2D has not, and is refused (see the definition).
     */
    void label_cleanup(const std::string& where);
    /**
     * @name The level set out of reach
     * The last clean-up's non-band triangles wholly inside the level set (the one-sided side
     * test) that the held objects' test kept out of the band. At a settled turn and at the loop's
     * end check_level_set_reachable() throws on them (warns under throw_on_unreachable_level_set
     * false), as in 3D.
     * @{
     */
    void check_level_set_reachable(const std::string& when);
    size_t m_unreachable_cells = 0;
    std::string m_unreachable_where;
    /// The last settle test's verdict (settle_refinement_turn()).
    bool m_turn_settled = false;
    /** @} */
    /**
     * @name The held objects
     * The 2D twins of TopoOffsetTetMesh's: the input complex, the held points (the base's
     * anchors), the region boundaries (the curve group's edges among them: 2D tracks and holds
     * the curve group as it does a region boundary) and the box (the mesh boundary) -- the
     * objects the optimization keeps the construction's topology on (label_cleanup()'s held
     * objects' test). In 2D a held object is a point or a curve: the codimension-1 held simplices,
     * which part the triangles around a vertex into chambers, are edges (3D: faces), and a held
     * point is a vertex (3D's held curves and points: edges and vertices). A held edge is on the
     * mesh boundary, an edge of a face of the input complex, or a tracked region boundary; a held
     * vertex is on the input complex, an anchor, or a corner of a held edge; so the held
     * simplices are closed under taking faces. Read off what 2D's operations carry -- the face
     * labels, m_is_on_input and the edge tracking -- not off the construction labels of edges and
     * vertices, which 2D's operations do not carry (split_adjust_position()), unlike 3D's
     * (close_labels()).
     * @{
     */
    bool held_object_edge(const Tuple& e) const;
    bool held_object_vertex(size_t v) const;
    /// DEBUG_held_topology: per label, band then non-band, the number of connected components
    /// and the Euler characteristic of the held simplices it touches, one copy per chamber (the
    /// triangles around a simplex joined across edges that are not held; see label_cleanup()).
    /// With `vertex_touch`, also each held vertex's position and whether the band, the non-band,
    /// touches it in some chamber. With `by_kind`, the same restricted to each kind of held
    /// object: the input complex, the held points, the region boundaries, the box. As in 3D.
    std::array<int64_t, 4> held_topology_signature(
        std::map<std::array<double, 2>, std::array<bool, 2>>* vertex_touch = nullptr,
        std::array<std::array<int64_t, 4>, 4>* by_kind = nullptr) const;
    /// DEBUG_held_topology: the signature against the last one; a change is logged, naming
    /// `pass` and the turn. The first call (after the construction) is the baseline.
    void held_topology_check(const std::string& pass);
    std::optional<std::array<int64_t, 4>> m_held_topology;
    std::map<std::array<double, 2>, std::array<bool, 2>> m_held_vertex_touch;
    std::array<std::array<int64_t, 4>, 4> m_held_topology_by_kind{};
    mutable std::vector<std::pair<size_t, Vector2d>> m_held_curve_band_components;
    mutable std::array<size_t, 2> m_held_curve_counts{}, m_held_curve_counts_before{};
    size_t m_held_topology_changes = 0;
    /** @} */

    /// init_optimize: optimize_offset() opens with a stencil_order loop without refinement. Set
    /// by marching_tris(), only when the target is beyond the maximum marchable distance.
    bool m_init_optimize = false;
    /// Leads every debug frame label: i during the init_optimize loop, g<k> during growth_mode
    /// "linear"'s stage k < N, s0 / s<L>a<A> during growth_mode "smart"'s level 0 / level L's
    /// attempt A, empty otherwise. As in 3D.
    std::string m_frame_prefix;
    /// growth_mode "linear": stages k = 0..N (N = linear_growth_steps), stage k at
    /// growth_stage_target(m_growth_start, target_distance, k, N). Before each stage
    /// target_distance is set to its target and the field rebuilt on it (init_offset_potential());
    /// stages before the last are optimize_offset_loop(false, true) to convergence, their frames
    /// led by g<k>; stage N is the ordinary loop. A stage before the last that does not converge
    /// stops the run there, target_distance and the field set back for the verdict. As in 3D.
    void linear_growth();
    /**
     * @brief growth_mode "smart": which target to try next from the last converged distance,
     * and what an attempt's result means. Pure, so it is unit-tested apart from the mesh. The 2D
     * twin of TopoOffsetTetMesh::SmartGrowthSchedule, the same rule.
     *
     * From `base` (d_0, then each converged distance) the j-th try after j failures is at
     * base + (delta - base) / 2^j: delta itself first. A success at delta is the end; a success
     * below it becomes the new base (j and the failures start again); `max_fails` failures from
     * one base give up.
     */
    struct SmartGrowthSchedule
    {
        double base; ///< the last converged distance
        double delta; ///< target_distance
        int max_fails; ///< smart_growth_max_fails
        int j = 0; ///< failed attempts from base, so halvings of the step
        int level = 1; ///< level 0 is the loop at d_0; each success below delta adds one
        double target() const { return j == 0 ? delta : base + (delta - base) / std::ldexp(1., j); }
        enum class Next { Done, Advance, Retry, GiveUp };
        Next report(const bool converged)
        {
            if (converged) {
                if (j == 0) return Next::Done;
                base = target();
                j = 0;
                ++level;
                return Next::Advance;
            }
            ++j;
            return j >= max_fails ? Next::GiveUp : Next::Retry;
        }
    };
    /// growth_mode "smart" (see the spec doc): the loop at d_0 up to max_rounds turns (throws when
    /// it does not converge), then attempts of smart_growth_attempts turns each, as
    /// SmartGrowthSchedule picks their targets, each judged converged and free of offset edges
    /// turned from the field (optimize_offset_loop(..., turned_free true)). The mesh is stored
    /// (take_snapshot()) after level 0 and after every success, and put back (restore_snapshot())
    /// after every failure; SmartGrowthSchedule::Next::GiveUp throws. The attempt at
    /// target_distance is the final stage: on success it runs the final pass and decides the
    /// verdict. As in 3D.
    void smart_growth();
    /**
     * @brief An in-memory image of the mesh, to put it back as it was (for growth_mode "smart":
     * the revert after a failed attempt). The 2D twin of TopoOffsetTetMesh::MeshSnapshot. Taken
     * on a consolidated mesh: the faces in fid order, each as oriented_tri_vids() gives it -- the
     * order the edge slots (3 fid + j) are keyed on -- and copies of every per-element collection
     * over the live slots: the vertex group (m_vertex_attribute, m_vertex_extra), the edge group
     * (m_edge_attribute, m_edge_extra) and the face group (m_face_attribute, m_face_extra). 2D has
     * no cell collection apart from the face group, which holds what 3D's m_tet_attribute does.
     */
    struct MeshSnapshot
    {
        size_t n_vertices = 0;
        std::vector<std::array<size_t, 3>> faces;
        std::vector<VertexAttributes> vertex;
        std::vector<VertexExtra2d> vertex_extra;
        std::vector<EdgeAttributes> edge;
        std::vector<EdgeExtra2d> edge_extra;
        std::vector<FaceAttributes> face;
        std::vector<FaceExtra2d> face_extra;
    };
    /// Consolidate, record the image, and rebuild the mesh from it at once (restore_snapshot()):
    /// the mesh then IS what a restore gives, so every attempt from the image -- the first, and
    /// each one after a revert -- starts from identical storage. As in 3D, where the rebuild sorts
    /// each vertex's tet fan, which consolidation leaves in its old order; 2D keeps every fan
    /// sorted, and here the rebuild resets what consolidation keeps: each face's hash, and every
    /// collection's size beyond the capacity. Refused with feature-point tracking on: its
    /// collection is not in the image. 3D refuses feature-edge tracking too; 2D has none, its
    /// tracked curves being the edge group's own tags, which are. Between passes only.
    MeshSnapshot take_snapshot();
    /// Put the mesh back as `s` holds it: the connectivity rebuilt (init_with_isolated_vertices(),
    /// which clears the old one and resizes every collection), the collections' live slots copied
    /// back, the band regions re-derived. Not restored: run counters, the debug-frame sequence.
    /// Between passes only. As in 3D.
    void restore_snapshot(const MeshSnapshot& s);
    /// Stage k of n's target: d0 + k (delta - d0) / n, so d0 at k = 0, and delta itself at k = n
    /// (not the formula, which rounding can leave a hair off it). As in 3D.
    static double growth_stage_target(double d0, double delta, int k, int n);

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

    /// Edge tags, the tagged curves of a triangle mesh's edge block (triwild's and simwild's): bit
    /// j of an edge's input set is m_edge_tag_names[j]. The 3D twin is
    /// TopoOffsetTetMesh::m_face_tag_names. Each has a tag id (m_edge_tag_id[j]), past every other
    /// tag, whose mask bit and tube hold its edges (init_surfaces_and_boundaries(),
    /// build_boundary_envelopes()), and which a selection (offsets) can name (m_edge_tag_bit: id
    /// -> bit) -- unless a triangle tag, an output tag or the curve group has the name, as triwild
    /// names an input's region and its curve alike (SimWildMeshTri::m_edge_tag_names): the name
    /// then selects the region (3D refuses a face tag a tet tag has).
    std::vector<std::string> m_edge_tag_names;
    std::vector<int64_t> m_edge_tag_id;
    std::map<int64_t, int> m_edge_tag_bit;
    /// The edge tags of edge `eid`: the bits of its input set below m_edge_tag_names.size().
    InputSet edge_tag_set(const size_t eid) const
    {
        const size_t n = m_edge_tag_names.size();
        const InputSet all = n >= 64 ? ~InputSet(0) : (InputSet(1) << n) - 1;
        return m_edge_attribute[eid].input_set() & all;
    }
    /// Point labels, the input points' (a tagged mesh's point block): bit j is the base's
    /// m_point_names[j], each name a tag id a selection (offsets) can name (m_point_tag_bit: id ->
    /// bit), past every other tag and with no mask bit (m_tag_bit), an edge tag's id where the two
    /// share a name, as in 3D. Where 2D differs: a label a triangle tag or the curve group already
    /// names keeps apart, not selectable by the name, which selects the region (3D refuses the
    /// name, "is also a tet or face tag"): triwild names an input's region, its curve and its
    /// points alike, so 3D's refusal would refuse every triwild output with points in a closed
    /// input. Every point is held and written back either way (hold_curves_and_points()).
    std::map<int64_t, int> m_point_tag_bit;
    /// The construction label of vertex `v` (VertexExtra2d::label) from what 2D's operations
    /// carry: 1 if it is on the input complex (m_is_on_input: a split's midpoint the AND of its
    /// ends, a collapse's survivor the OR), else 2 if a band triangle (face label 2, carried) has
    /// it as a corner, else 0 -- the closure the construction labels by (marching_tris()). A
    /// swap or a smoothing step changes none of the three. Until 2026-11-11 the optimization left
    /// the label as the slot held it: MEASURED on a curve offset (the curve group or "A & B", one
    /// turn), the result's input complex (labels 1) missed the 4 of its 9 vertices the split pass
    /// made, which carry m_is_on_input. The AND over-claims on a chord between two complex
    /// vertices that is no edge of the complex; MEASURED over "[2d]~[.]" (52 cases), 0 of the 16
    /// splits with both ends on the complex were such a chord.
    int carried_vertex_label(size_t v) const
    {
        if (m_vertex_extra[v].m_is_on_input) return 1;
        for (const size_t fid : get_one_ring_fids_for_vertex(v)) {
            if (m_face_extra[fid].label == 2) return 2;
        }
        return 0;
    }
    /// Whether vertex `vid` is an input point of label `bit`: by VertexExtra2d::point_inputs until
    /// hold_curves_and_points() anchors the points, by its anchor's labels after. As
    /// TopoOffsetTetMesh::vertex_has_point().
    bool vertex_has_point(size_t vid, int bit) const;

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
     * @brief THE per-face energy E(t), read from the mesh as it is -- the face's corners and its
     * label, nothing hypothetical. E = the sum of E(t) over the live faces is the one energy of
     * the 2D loop. The 2D twin of TopoOffsetTetMesh::tet_energy(), term for term:
     *
     *     tri_energy(t) = cell_energy(oriented_tri_vids(t), get_quality(t),
     *                                 face_is_offset_band(t))
     *
     *     cell_energy(vids, A, band) = MAX_ENERGY                          if A >= MAX_ENERGY
     *                                = (w / s^3) area A^3 + [band] b(vids)  otherwise
     *
     * w = offset_amips_weight, s = stop_energy (optimization_stop_metric()), so a face at the stop
     * energy costs about w area. A is the base's TriOptimizerMesh::get_quality(): the AMIPS the 2D
     * engine stores as the face quality and compares with stop_energy, the first power (3D's
     * engine stores AMIPS^3 and compares its cube root); its MAX_ENERGY (unscoreable) is the
     * face's energy. area A^3 is area_amips3(), from the exact AMIPS and the edge lengths, never
     * from a determinant. b is band_cell_term(): the band face's part of int_B (d - delta) / delta
     * dA, B the union of the band faces -- exactly ("exact"), or as an upper bound that no split
     * raises ("corner_bound"). Both terms are areas, so E converges under refinement. Before the
     * march, while the repulsion runs (m_repulsion_potential set), a face also carries
     * repulsion_cell_term(); there is no band then. A face's energy depends on its own three
     * corners and its label alone: an operation changes the energy of the faces it changes and
     * of no other.
     *
     * WHY THE CUBE, AS IN 3D, AND NOT THE STORED QUALITY'S FIRST POWER. At fixed edge lengths
     * AMIPS2D = tr(J^T J) / det J and area = (sqrt3/4) det J, so area A^p grows like A^(p-1) as a
     * face flattens: p = 1 (area times the stored quality, the literal reading of 3D's "vol times
     * the stored AMIPS^3") is constant there and no barrier; p = 3 grows like A^2 (3D's vol A^3
     * like A^(3/2)). 3D MEASURED the non-barrier (EXPERIMENTAL_amips_pow, 679a0f7ead, removed):
     * vol AMIPS, which falls as a cell flattens, let the pre-march TetWild pass's first smoothing
     * pass take max AMIPS from 19.9 to 8e15 on 126196.
     *
     * WHERE IT IS COMPARED: always as the SUM of tri_energy() over the faces an operation changes,
     * before against after, the faces summed in the same order each time:
     *   - Collapse: before = the sum over v1's ring (collapse_before_vertex()); after = the sum
     *     over the survivor's ring less v2's old faces outside v1's ring, which keep their corners
     *     and labels (collapse_edge_after()). Refused unless after <= before (a tie passes, a NaN
     *     refuses). Not in coarsening.
     *   - Swap (the 2D flip, (a,b,c) + (a,b,d) -> (a,c,d) + (b,c,d), always interior: the engine
     *     refuses every tracked edge, so both faces share one label, which the new faces keep):
     *     before = the sum over the two faces (swap_edge_before()); refused unless the sum over
     *     the two new faces is STRICTLY below it (swap_edge_after()). Strict, so a swap pass
     *     terminates.
     *   - Smoothing (smooth_front_vertex(), smooth_nonfront_vertex(), smooth_repulsion_vertex()):
     *     the sum over the vertex's ring after the move may not exceed the sum before.
     *   - Split: no rule, unless EXPERIMENTAL_split_energy_gate (the children's sum strictly below
     *     the parents').
     * The engine's own AMIPS rules for collapses and swaps are off: collapse_quality_allowed()
     * and swap_quality_allowed() admit everything, and this energy decides in their place.
     *
     * THE SMOOTHER minimises the vertex's part of E (smoothing_extra_energy()): the AMIPS term
     * over its ring (amips3_energy()) and, unless an input envelope holds the vertex on the input
     * (vertex_is_held_on_input()), the band term of its band faces (band_volume_energy_at()).
     * Every move is then judged by the ring's sum of this energy: a vertex held on a region
     * boundary through the engine's projected step, which takes the first projected position
     * where that sum drops (smoothing_ring_energy()); every other vertex through the veto after
     * its solve (smooth_front_vertex(), smooth_nonfront_vertex()). As in 3D.
     *
     * WHERE 2D DIFFERS FROM 3D'S RULES, and why:
     *   - No TetWild rules (3D's m_op_rules) and no TetWild pass (3D's m_tetwild_pass): 2D has no
     *     optimize_construction pass, and its final pass (m_freeze_front) is the frozen-front pass
     *     3D replaced; its operations read this energy, band term included.
     *   - BAND TERM THAT CANCELS (band_term_cancels()) is false for every operation but under
     *     "exact_swept", where every rule adds the band term's exact change instead (EXACT SWEPT
     *     CHANGE below), as in 3D.
     *   - use_rest_pose: no 2D plastic face reads a rest here; 2D's plastic medium is
     *     deform_others', hard-wired off (envelope_setup()).
     */
    double tri_energy(size_t fid, bool band_term = true) const;
    /// Whether an operation whose old faces are `fids` leaves the band term unchanged exactly, so
    /// its sums leave it out; see TopoOffsetTetMesh::tet_energy(), BAND TERM THAT CANCELS.
    ///  - "exact_swept": true, as in 3D: every operation's sums leave the band term out, and its
    ///    rule adds the band term's exact change (EXACT SWEPT CHANGE below).
    ///  - "exact": false. 3D also needs every old and new cell clear of the input's curved
    ///    bisectors, which it tests with the integral's front-flux reach (ExactBandIntegral::
    ///    prepare_front_flux(), front_flux_flag()); ExactBandIntegral2D has none, being
    ///    "front_flux"'s machinery, which is not ported. Without it every sum carries the band
    ///    term, which is E itself. 3D MEASURED reading it as 0 (cube, target 1e-2, bar 1e-4,
    ///    front_flux, 15 turns, serial): 0 of those decisions flipped, 19% of the run saved.
    ///  - "corner_bound": false, as in 3D, its per-face bound changes with the faces.
    bool band_term_cancels(const std::vector<size_t>& /*fids*/) const { return swept_rule(); }
    /**
     * EXACT SWEPT CHANGE (EXPERIMENTAL_band_volume_rule "exact_swept", 3D's default since
     * 455ab9ba66): the 2D twin of TopoOffsetTetMesh's. The energy is "exact"'s -- b(t) = J_t /
     * delta, J_t = ExactBandIntegral2D::value() -- but an operation's rule does not integrate its
     * faces before and after: it compares the AMIPS terms of the faces it changes
     * (band_term_cancels() is true for every one) plus the CHANGE of the band term E* / delta,
     * E* = int_B (d - delta) dA, B the union of the band faces, which lives only where B changes
     * and is integrated exactly there by value() on small triangles:
     *   - a vertex v moving from x0 to x1 (smoothing, a collapse's v1 moving onto v2): dE* = sum
     *     over the edges f of the boundary of B through v of sign_f value(conv(x0, x1, a_f)), a_f
     *     f's other end, sign_f = +1 where B grows (x1 beyond f, away from the band face f
     *     bounds). Why: a band face t = [v, a, p] moved to [x1, a, p] differs from [x0, a, p] by
     *     the 2-chain [x0, x1, a] - [x0, x1, p] (the boundary of the 3-simplex [x0, x1, a, p] is
     *     a 2-cycle, and a 2-cycle in the plane integrates every function to 0), one oriented
     *     triangle per edge of t through v, each edge as it bounds t. Over the band faces the
     *     edges between two of them cancel; the boundary edges of B are left, each with the
     *     oriented triangle [x0, x1, a_f], whose integral is sign_f times the unsigned one. Exact
     *     for mixed signs and for any move that inverts no face; a collapse's faces through v2
     *     become flat and drop out the same way. A vertex on no boundary edge of B: 0.
     *   - a swap: 0. 2D's only swap is the interior flip (the engine refuses every tracked edge,
     *     so both faces share one label, which the new faces keep), which keeps B's region and
     *     boundary; 3D's band_chain_change() serves its surface flips, which 2D does not have.
     *   - a split: 0 (the children tile the parents, labels inherited).
     * The decisions read these changes; tri_energy(), the label clean-up and every whole-mesh sum
     * read "exact"'s J_t. Where value() takes rule (a)'s bound on a curved piece, the bound
     * depends on the face, and the swept change is not the difference of the faces' bounds (3D
     * MEASURED, every accepted operation's swept change against the change of its cells' sum of
     * value(): within 1.2e-18 on the cube, up to 2.1e-3 where a rule-(a) piece is involved).
     *
     * Several entries (Parameters::offsets): each band face t integrates its own piece's
     * integrand, r(t) = place_piece() of its corners (band_cell_term()), so E* / delta is the sum
     * over the pieces r of E*_r / delta_r, E*_r = int_{B_r} (d_r - delta_r) dA, B_r the band
     * faces of piece r, and the change is the sum of each B_r's:
     *   - a vertex move: the boundary edges of B_r through v -- the face across not a band face
     *     of piece r, so an edge between band faces of two pieces bounds both -- each swept
     *     triangle integrated as its face's piece (band_swept_edges(), one list per piece, the
     *     pieces read where the move starts). A face whose piece the operation changes (its
     *     corners' pieces, a collapse replacing one; or, where no corner has one, nearest_piece()
     *     of its first corner's position) adds b_{r1}(t) - b_{r0}(t) on the face after the
     *     operation (band_relabel_change()).
     *   - a swap whose faces, old and new, are not all of one piece: the change of its faces'
     *     own terms, sum of b(t) over the new less the old.
     * With one entry every face is piece -1, and every change is the one above, bit for bit.
     * MEASURED (several-offsets-optimize-distances-2d, its case a one square at 0.1, b one at
     * 0.3, 0.5 apart), each change against the change of its faces' band_cell_term(): with the
     * moving vertex's piece read for every edge and an edge between band faces of two pieces
     * read as inside B, one smoothing move was off -- vertex 309, made since the pieces were
     * assigned, read a's piece (nearest_piece() of its position) while its three band faces
     * read b's: -3.81e-3 against their +4.12e-5, accepted -- and b's front ended at 0.278
     * against the check's 0.285 ("exact" passes); in several-offsets-growth-2d 12 of 353
     * collapses in one pass (7 decided otherwise). Now none is off (to 1e-9 of the terms), and
     * both pass.
     *
     * The projected step (a vertex held on a region boundary, front or not) takes each
     * candidate's swept change in the engine's ring hook (smoothing_ring_energy()), as 3D's.
     * Until 2026-11-12 the 2D engine had no such hook, and the position its projected step kept
     * was tested after the step, at that position alone: 3D's arrangement before its hook,
     * measured there: 5,270 of 21,687 held moves refused (cube, 7 turns).
     */
    /// "exact_swept" (EXACT SWEPT CHANGE).
    bool swept_rule() const { return m_offset_params.band_volume_rule == "exact_swept"; }
    /// EXACT SWEPT CHANGE: dE* / delta for vid moving from x0 to x1, the boundary edges of B
    /// through vid read from the mesh (labels, connectivity, the other corners' positions), each
    /// piece's faces as theirs; the change of a face's piece aside (band_relabel_change()). `st`
    /// collects value()'s statistics (rule-(a) pieces) when given. As 3D's.
    double band_swept_change(
        size_t vid,
        const Vector2d& x0,
        const Vector2d& x1,
        ExactBandIntegral2D::Stats* st = nullptr) const;
    /// EXACT SWEPT CHANGE, several entries: the boundary edges of B_r through a vertex, B_r the
    /// band faces of piece r (place_piece() of the face; -1, the only one, with one entry).
    struct SweptEdgesOfPiece
    {
        int piece = -1;
        std::vector<BandVolumeEnergy2D::SweptEdge> edges;
    };
    /// band_swept_change() in two halves, as 3D's band_swept_faces() and band_swept_sum(): the
    /// boundary edges of each B_r through vid, each ordered out of B_r at x0 (no value() call),
    /// one list per piece in the order the ring first meets it, and the sum over them for a move
    /// x0 -> x1, each list at its piece's distance and integral. A boundary edge of B_r has one
    /// face of B_r at it, the face across it not one (another piece's, no band face, or none:
    /// the domain wall). The pieces are read on the mesh as it is (place_piece()).
    void band_swept_edges(size_t vid, const Vector2d& x0, std::vector<SweptEdgesOfPiece>& pieces)
        const;
    double band_swept_sum(
        const Vector2d& x0,
        const Vector2d& x1,
        const std::vector<SweptEdgesOfPiece>& pieces,
        ExactBandIntegral2D::Stats* st = nullptr) const;
    /// EXACT SWEPT CHANGE: the band faces among fids with their pieces (place_piece()), sorted,
    /// read before an operation for band_relabel_change().
    std::vector<std::pair<size_t, int>> band_face_pieces(const std::vector<size_t>& fids) const;
    /// EXACT SWEPT CHANGE: the change of the band term where an operation changed a face's piece:
    /// the sum over the band faces t among fids whose piece r1 now is not the r0 `before` gives
    /// them of b_{r1}(t) - b_{r0}(t), t as it now is; 0 with one entry.
    double band_relabel_change(
        const std::vector<size_t>& fids,
        const std::vector<std::pair<size_t, int>>& before) const;
    /// "exact_swept": a swap's band term change, the faces it made `made` (EXACT SWEPT CHANGE):
    /// 0 where its faces, old (m_swap_old_band) and new, are all of one piece -- 2D's only swap
    /// is the interior flip, which keeps B_r's region and boundary -- and the sum of the new
    /// band faces' band_cell_term() less the old's, each at its piece, where they are not.
    double swap_swept_change(const std::vector<size_t>& made) const;
    /// "exact_swept": the smoother's acceptance after a solve moved vid from x0 to where it is,
    /// 3D's swept_accept(). The move stands iff the ring's AMIPS change (tri_energy_sum(ring,
    /// false) - amips_before) plus band_swept_change() is <= 0 (a tie passes, a NaN refuses); if
    /// not, the step is halved toward x0 and tested again, up to kSweptHalvings times, then
    /// refused. The halved positions are on the segment between two valid positions, and the
    /// positions at which vid's ring has no inverted face are an intersection of half-planes, so
    /// they are valid too (tested anyway, exactly). Counts into c. At a position where the solve's
    /// search model took its last value, the model's value is the change
    /// (BandVolumeEnergy2D::swept_value_at(), the same function of the same inputs), each piece's
    /// model its piece's. The edges and the faces' pieces are read with vid at x0, where the move
    /// starts.
    static constexpr int kSweptHalvings = 4;
    struct SweptCounters
    {
        std::atomic<size_t> solves{0}, refused{0};
        std::array<std::atomic<size_t>, kSweptHalvings + 1> accepted{};
    };
    bool swept_accept(
        size_t vid,
        const std::vector<size_t>& ring,
        const Vector2d& x0,
        double amips_before,
        SweptCounters& c);
    mutable SweptCounters m_swept_front, m_swept_nonfront;
    /// "exact_swept", the projected step: the vertex the smoother is placing, and at the `stored`
    /// call of smoothing_ring_energy() its position, its boundary edges of each B_r
    /// (band_swept_edges()) and its band faces' pieces (band_face_pieces()), which every
    /// candidate's band change is taken from. As 3D's.
    struct SweptStored
    {
        size_t vid = 0;
        Vector2d x0 = Vector2d::Zero();
        std::vector<SweptEdgesOfPiece> pieces;
        std::vector<std::pair<size_t, int>> face_pieces;
    };
    mutable wmtk::threading::enumerable_thread_specific<SweptStored> m_swept_stored;
    /**
     * @brief What the shared smoother's projected step compares for a vertex held on a region
     * boundary: the sum of tri_energy() over the one-ring, the rule every other move answers to;
     * under "exact_swept" the ring's AMIPS terms plus the band term's exact change from the
     * position at the `stored` call (EXACT SWEPT CHANGE). The step takes the first projected
     * position where it is strictly below its value before the move. Read on the mesh as it is in
     * both calls (tri_energy() reads the corners' positions). As 3D's (8c2416e117); 2D has no
     * TetWild rules and refuses the "surface" rule, so neither of 3D's other branches exists here.
     */
    double smoothing_ring_energy(const std::vector<size_t>& ring, const bool stored) const override
    {
        double s = 0.;
        if (!swept_rule()) {
            for (const size_t fid : ring) s += tri_energy(fid);
            return s;
        }
        // the vertex is the one smooth_nonfront_vertex() or smooth_front_vertex() is smoothing,
        // the same in both calls
        for (const size_t fid : ring) s += tri_energy(fid, false);
        auto& st = m_swept_stored.local();
        const size_t vid = st.vid;
        const auto vs = oriented_tri_vids(ring.front());
        if (std::find(vs.begin(), vs.end(), vid) == vs.end()) {
            log_and_throw_error("exact_swept: the projected step's ring is not vertex {}'s", vid);
        }
        if (stored) {
            // the edges and the faces' pieces where the move starts (swept_accept())
            st.x0 = m_vertex_attribute[vid].m_posf;
            band_swept_edges(vid, st.x0, st.pieces);
            st.face_pieces = band_face_pieces(ring);
            return s;
        }
        return s + band_swept_sum(st.x0, m_vertex_attribute[vid].m_posf, st.pieces) +
               band_relabel_change(ring, st.face_pieces);
    }
    /// "exact_swept": the search models band_volume_energy_at() built last on this thread, one per
    /// piece in band_swept_edges()'s order, for swept_accept(). As 3D's.
    mutable wmtk::threading::enumerable_thread_specific<
        std::vector<std::shared_ptr<const BandVolumeEnergy2D>>>
        m_swept_model;
    /// "exact_swept": the smoother's search model's derivative evaluations, by kind, per pass
    /// (BandVolumeEnergy2D::set_swept()). As 3D's.
    mutable BandVolumeEnergy2D::SweptDerivCounts m_swept_derivs;
    /// area AMIPS^3 of a valid face from its AMIPS and edge lengths, never from a determinant: the
    /// 2D twin of TopoOffsetTetMesh::vol_amips3(). AMIPS2D = tr / det(J), J the map from the unit
    /// equilateral triangle, gives area = (sqrt3/4) det J = (sqrt3/4) tr / AMIPS, so area AMIPS^3
    /// = (sqrt3/4) tr AMIPS^2, tr = tr(J^T J) = (2/3) sum over the three edges of |e|^2 (no
    /// cancellation). See vol_amips3() for the 3D measurement: a nearly flat tet whose double
    /// determinant came out 0 was priced at 0 against an exact 1.7e21.
    double area_amips3(const std::array<size_t, 3>& vids, const double amips) const
    {
        double tr = 0.;
        for (int i = 0; i < 3; ++i) {
            tr += (m_vertex_attribute[vids[size_t(i)]].m_posf -
                   m_vertex_attribute[vids[size_t((i + 1) % 3)]].m_posf)
                      .squaredNorm();
        }
        tr *= 2. / 3.;
        return std::sqrt(3.) / 4. * tr * amips * amips;
    }
    /**
     * @brief b(t), one band face's part of int_B (d - delta) / delta dA. The 2D twin of
     * TopoOffsetTetMesh::band_cell_term(), d the distance to the input's segments and points
     * (ExactBandIntegral2D's primitives), delta = target_distance:
     *   - "exact" (and "exact_swept", whose energy it is): J_t / delta, J_t = int_t (d - delta) dA
     *     in closed form (ExactBandIntegral2D::value()) -- exact up to rounding, an upper bound
     *     only on the pieces where an edge's and a vertex's regions meet on a parabola (rule (a)).
     *   - "corner_bound": area times the min over ALL input primitives P of the corner mean of
     *     (d_P - delta) / delta (band_cell_corner_bound()) -- an upper bound on the exact part (d
     *     <= d_P, and the convex d_P lies below its linear interpolant) that no split raises; see
     *     the 3D twin for the measurements behind the corner bound and the min over all.
     * "front_flux" and "surface" are not ported, and are refused here, at the first band face,
     * rather than read as another rule: "front_flux"'s front form changes how the smoother
     * evaluates the band term (until 2026-11-12 2D ran it as "exact", which its doc does not
     * say); "surface" puts no term on a face (3D: cell_energy() skips it, every rule adding the
     * front's face terms beside), and its face terms are not ported. The face's own piece's
     * distance and integral (place_piece(); the run's and the whole complex's with one entry), as
     * 3D's.
     */
    double band_cell_term(const std::array<size_t, 3>& vids) const
    {
        if (band_rule_exact()) return band_cell_term(vids, place_piece(vids.data(), 3));
        int64_t best = -1;
        return band_cell_corner_bound(vids, best);
    }
    /// band_cell_term()'s J_t / delta ("exact" and the rules that take its term) at piece r's
    /// distance and integral, whatever the face's piece: EXACT SWEPT CHANGE's change of a face's
    /// piece.
    double band_cell_term(const std::array<size_t, 3>& vids, const int r) const
    {
        const std::array<Vector2d, 3> p = {
            {m_vertex_attribute[vids[0]].m_posf,
             m_vertex_attribute[vids[1]].m_posf,
             m_vertex_attribute[vids[2]].m_posf}};
        const double delta = region_delta(r);
        return band_exact_for(r)->value(p, delta) / delta;
    }
    /// The band rule band_cell_term() and the smoother's band term (band_volume_energy_at())
    /// read: true for "exact" and "exact_swept", which takes its term, false for "corner_bound";
    /// any other rule ("front_flux", "surface") has no 2D port and throws.
    bool band_rule_exact() const
    {
        const std::string& rule = m_offset_params.band_volume_rule;
        if (rule == "exact" || rule == "exact_swept") return true;
        if (rule != "corner_bound") {
            log_and_throw_error(
                "EXPERIMENTAL_band_volume_rule \"{}\" has no 2D port: the 2D energy takes "
                "\"corner_bound\", \"exact\" or \"exact_swept\"",
                rule);
        }
        return false;
    }
    /// band_cell_term()'s corner bound, with its minimising primitive in `best`: the face's
    /// unsigned area (the face is valid wherever this is read) times (md - delta) / delta, md the
    /// min mean (ExactBandIntegral2D::min_mean_distance(), 3D's min_mean_distance() on its input
    /// envelope, which 2D does not have) on the face's piece.
    double band_cell_corner_bound(const std::array<size_t, 3>& vids, int64_t& best) const
    {
        const int r = place_piece(vids.data(), 3);
        const double delta = region_delta(r);
        std::array<Vector2d, 3> p;
        for (int j = 0; j < 3; ++j) p[size_t(j)] = m_vertex_attribute[vids[size_t(j)]].m_posf;
        const Vector2d u = p[1] - p[0], v = p[2] - p[0];
        const double area = std::abs(u.x() * v.y() - u.y() * v.x()) / 2.;
        const double md = band_exact_for(r)->min_mean_distance(p.data(), 3, best);
        return area * ((md - delta) / delta);
    }
    /// The AMIPS term's weight, w / s^3 (w = offset_amips_weight, s = stop_energy): the term is
    /// w area (AMIPS / s)^3, about w area for a face at the stop energy. As in 3D.
    double band_amips_weight() const
    {
        const double s = optimization_stop_metric();
        return m_offset_params.offset_amips_weight / (s * s * s);
    }
    /// One face's energy from its corners, its AMIPS and its band label; see tri_energy().
    double cell_energy(const std::array<size_t, 3>& vids, const double amips, const bool band) const
    {
        if (amips >= MAX_ENERGY) return MAX_ENERGY;
        double a = band_amips_weight() * area_amips3(vids, amips);
        if (band) a += band_cell_term(vids);
        if (m_repulsion_potential) a += repulsion_cell_term(vids);
        return a;
    }
    /// The sum of tri_energy() over `fids`, in their order: what every rule compares.
    double tri_energy_sum(const std::vector<size_t>& fids, bool band_term = true) const;
    /// E, the sum of tri_energy() over every live face (get_faces() order): the [band volume
    /// energy] lines' and the label clean-up's. 3D's global_energy(), without its "surface" face
    /// terms, which 2D refuses (band_cell_term()).
    double global_energy() const;
    /// The offset energy: band_cell_term() summed over the band faces, E without its AMIPS term.
    /// What the settle test compares (settle_metric(), settle_refinement_turn()): E's AMIPS term
    /// changes wherever the mesh is improved, the front moving or not. As in 3D, whose
    /// measurement motivates it (2026-10-14, the cube outward: splits raised E through the AMIPS
    /// term by 0.7-3.6 bar energies per turn).
    double offset_energy() const;
    /// E's two parts over the live faces, {the AMIPS term (with the repulsion term, 0 in the
    /// loop), the band term}, summed in get_faces() order; E is their sum. A band face is
    /// evaluated as cell_energy(vids, A, false) and band_cell_term(vids) apart, not tri_energy()
    /// and band_cell_term(), which would evaluate the band term twice. 3D's energy_parts(); where
    /// 2D differs: serial (3D evaluates the cells on its threads, diagnostic_parallel_for(), and
    /// sums them in the same order; 2D has no such helper).
    std::pair<double, double> energy_parts() const;
    /// "[energy step] turn N <step>: E | AMIPS term | band term" (energy_parts()), the line
    /// ~/Developer/wmtk_outputs/plot_energy.py plots. Logged at each operation group's start
    /// ("start", a baseline), as in 3D. Where 2D differs: 3D also logs one per smoothing pass from
    /// the engine's end-of-pass hook (TetOptimizerMesh::smoothing_pass_done()), which the 2D
    /// engine has not; the turn is passed for the same reason (3D keeps it in a member the hook
    /// reads).
    void log_energy_step(int turn, const char* step) const;
    /// What the settle test compares turn to turn: offset_energy(), as 3D's under the volume
    /// rules. 3D's other metrics -- O* (the EXPERIMENTAL_settle_test "stall" test with
    /// settle_stall_eps) and O_g ("bar") -- are the "surface" rule's, which Parameters.h refuses
    /// under every other rule and 2D refuses itself (band_cell_term()): they have no 2D twin
    /// until "surface" has one.
    double settle_metric() const { return offset_energy(); }
    /// 1 / front_conv_frac()^2: the factor that turns a squared relative error into a squared
    /// error in units of the tolerance. The weight of the front smoother's offset terms, and the
    /// scale of edge_offset_term(). `piece`: the place's (place_piece()), whose bar it is in; -1,
    /// the run's. As in 3D.
    double offset_term_weight(const int piece = -1) const
    {
        const double f = front_conv_frac_of(piece);
        return 1. / (f * f);
    }

    /**
     * @brief Place a vertex, keeping its exact and rounded coordinates in step.
     *
     * As in 3D: the offset works in doubles, so every vertex it places is rounded and its exact
     * position is its double one (set_pos_to_posf).
     */
    void set_vertex_position(const size_t vid, const Vector2d& p)
    {
        m_vertex_attribute[vid].m_posf = p;
        m_vertex_attribute[vid].set_pos_to_posf();
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
    /// The energy rules (see tri_energy()): collapses refused for raising E over the faces they
    /// change, swaps for not strictly lowering it, and under EXPERIMENTAL_split_energy_gate splits
    /// for not strictly lowering it. 3D's m_volume_reject_collapse / _swap / _split.
    mutable std::atomic<int> iter_cnt_collapse_energy_reject{0};
    mutable std::atomic<int> iter_cnt_swap_energy_reject{0};
    mutable std::atomic<int> iter_cnt_split_energy_reject{0};
    /// EXPERIMENTAL_split_energy_gate: the split in flight's parents' sum of tri_energy(), taken
    /// in split_edge_before() and compared in split_edge_after(). As in 3D.
    mutable wmtk::threading::enumerable_thread_specific<double> m_split_energy_before;
    /// Splits of an offset-boundary edge: offered, accepted. Real attempts only: a screened
    /// pass's dry runs are not counted.
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
     * point where d(x) = D, D = m_construction_distances[k] (entry k's) and d(x) the distance to
     * entry k's complex (offset_distance()). From t = 0 the trace evaluates d at the current
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
        size_t k,
        const Vector2d& p_in,
        const Vector2d& p_out,
        Vector2d& p_new,
        size_t& steps) const;
    /// marching_tris() tallies for the construction log: edges placed on the level set / at the
    /// midpoint because the trace left the edge, and the trace steps (total, max). Reset at the
    /// start of marching_tris().
    size_t m_marching_root_splits = 0, m_marching_midpoint_splits = 0;
    size_t m_marching_trace_steps = 0, m_marching_trace_steps_max = 0;
    /// The distance the marching's sphere trace aims for, per entry of Parameters::offsets: its
    /// target_distance, or half its maximum marchable distance under construction_mode
    /// "max_marchable_fallback" (see the marching). m_construction_traced[k] false: the entry's
    /// marched edges are split at their midpoints instead. m_construction_distance is the
    /// smallest, what the run-level readers take (growth_mode). As in 3D.
    std::vector<double> m_construction_distances;
    std::vector<bool> m_construction_traced;
    double m_construction_distance = 0.;
    /// growth_mode "linear" / "smart": d_0, the distance the march traced the front to when that
    /// is below target_distance (m_construction_distance, half the maximum marchable distance); 0
    /// when growth_mode does not apply (none, a target the march traces to, no edge marched, or
    /// the midpoint fallback). Several entries: the smallest entry's d_0 (m_growth_starts). Set by
    /// marching_tris(), read by optimize_offset(). As in 3D.
    double m_growth_start = 0.;

    /**
     * @brief Reject any collapse that violates the substructure link condition, and remember the
     * survivor's sizing for sizing_collapse_min = false.
     *
     * The base applies the link condition only when both endpoints already sit on a tracked
     * surface or the bbox, which is the right rule for tetwild and simwild but not here: the
     * offset region is a thin band, and a collapse with only one endpoint on the boundary can
     * still pinch its two sides together and make the region non-manifold. The offset asks
     * unconditionally. The energy rule's before-half is taken in collapse_before_vertex(). As in
     * 3D.
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
    /// The collapse energy rule's before-half, per thread (collapse_before_vertex()): the sum of
    /// tri_energy() over v1's ring, the faces the collapse reshapes or removes, and the faces of
    /// v2's ring outside it, sorted, which keep their slots, corners and labels and so their
    /// energy. The 3D twin is CollapseVolume, less its "surface" and use_rest_pose fields (not
    /// ported, see tri_energy()).
    struct CollapseVolume
    {
        double before = 0.;
        /// band_term_cancels() over v1's ring: both sums leave the band term out.
        bool band_cancels = false;
        std::vector<size_t> v2_only;
        /// "exact_swept": v1's boundary edges of each B_r (band_swept_edges()), the two
        /// positions, and the pieces of v1's band faces (band_face_pieces()), taken on the mesh
        /// before; the change is integrated in collapse_edge_after(), for the collapses that pass
        /// the engine's tests (3D MEASURED integrating it before the inversion test that refuses
        /// most attempts: the collapse pass 35.0 s against "exact"'s 3.6 s).
        std::vector<SweptEdgesOfPiece> swept_edges;
        Vector2d swept_x1 = Vector2d::Zero(), swept_x2 = Vector2d::Zero();
        std::vector<std::pair<size_t, int>> swept_pieces;
    };
    mutable wmtk::threading::enumerable_thread_specific<CollapseVolume> m_collapse_volume;

    /**
     * @brief Reject a flip whose new edge already exists, and cache the energy rule's
     * before-half.
     *
     * Flipping (a,b) to (c,d) when c and d are already joined creates a second edge between the
     * same pair. Across a thin offset band that is how the two sides get stitched together and the
     * region stops being manifold. The base refuses tracked-surface edges but not this.
     */
    bool swap_edge_before(const Tuple& t) override;

    /// THE ENERGY RULE for a swap (see tri_energy()): the sum of tri_energy() over the two faces
    /// the flip made must be STRICTLY below the sum over the two it replaced
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
    /// the veto, and how many it refused for raising the ring's sum of tri_energy(). Reported and
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
     * (4/3 l s-bar), whether it is force-split queued, and is_edge_on_surface (the engine never
     * swaps a surface edge). Plus a scan for
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
    /// The swap energy rule's before-half: the sum of tri_energy() over the two faces the flip
    /// replaces, cached by swap_edge_before() and compared by swap_edge_after(). A face's energy
    /// is its own corners' and label's, so no face outside the two changes energy. 3D's
    /// m_swap_energy_before; 3D's SwapRecord, which the 4-4 / 5-6 case search and the face swap's
    /// gate read candidate cells' labels from, has no 2D twin: 2D's only swap has no candidate
    /// cells, its rule reading the two new faces on the mesh.
    mutable wmtk::threading::enumerable_thread_specific<double> m_swap_energy_before;
    /// band_term_cancels() over the two faces, for both sums (3D's SwapRecord::band_cancels).
    mutable wmtk::threading::enumerable_thread_specific<bool> m_swap_band_cancels;
    /// "exact_swept": the band faces the flip replaces, each with its piece (place_piece()), for
    /// a flip whose faces are not all of one piece (EXACT SWEPT CHANGE; 3D's SwapRecord::old_band).
    mutable wmtk::threading::enumerable_thread_specific<
        std::vector<std::pair<std::array<size_t, 3>, int>>>
        m_swap_old_band;
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
     * @brief The fold predicate, the one test of what a fold is: the 2D twin of
     * TopoOffsetTetMesh::fold_test(), one dimension down. A vertex of the offset curve with
     * exactly two offset edges, their normals out of the band n0 and n1, is a fold when the
     * normals are within `angle` (foldover_angle) of opposite -- the two edges lying on top of
     * each other, the tip of a zero-area sheet -- and at least one of the two edges is turned
     * from the field: its angle to the field's normal, field_angle(0) or field_angle(1)
     * (front_edge_field_angle()), over `angle`, or not measurable (NaN), so that nothing
     * unmeasured passes for no fold. Two opposite edges that both follow the field are no fold but
     * a ridge vertex (3D's ridge edge): the front following a corner of the level set sharper
     * than `angle`, which is where it belongs. field_angle is asked only of an opposite pair. The
     * normals' orientation does not decide the opposite test: n0 . n1 is minus the cosine of
     * the angle between the two edges' directions from the vertex, whichever side the band is on,
     * so a sheet of the band and one of the background are caught alike.
     */
    enum class FoldTest { none, ridge_vertex, fold };
    template <typename FieldAngle>
    static FoldTest fold_test(
        const Vector2d& n0,
        const Vector2d& n1,
        const FieldAngle& field_angle,
        const double angle)
    {
        if (!(n0.dot(n1) < -std::cos(angle * M_PI / 180.))) return FoldTest::none;
        const double a0 = field_angle(0), a1 = field_angle(1);
        return !(a0 <= angle) || !(a1 <= angle) ? FoldTest::fold : FoldTest::ridge_vertex;
    }
    /// The vertices of the offset curve with exactly two offset edges, sorted by fold_test(): the
    /// folds, and the ridge vertices with the largest field angle among their edges.
    /// `fold_vertices`, when given, receives every fold's vertex. 3D's count_folds(), whose
    /// folds are edges with two offset faces; as there, an offset edge on the domain boundary is
    /// left out (3D's offset_face_normal_out_of_band() reads a face's two cells).
    struct FoldCount
    {
        size_t folds = 0, ridge_vertices = 0;
        double ridge_max_angle = 0.;
    };
    FoldCount count_folds(std::vector<size_t>* fold_vertices = nullptr) const;
    /// The band face carrying offset edge (a, b); -1 for none. 3D's front_face_band_cell().
    int64_t front_edge_band_face(size_t a, size_t b) const;
    /// Offset edge (a, b)'s unit normal in `n`, turned out of its band face (away from the face's
    /// third corner). False, `n` unusable, where it cannot be measured: a degenerate edge, or no
    /// band face. 3D's front_face_out_normal().
    bool front_edge_out_normal(size_t a, size_t b, Vector2d& n) const;
    /// front_edge_out_normal() with the edge's band face given: `band_fid`, a band face holding
    /// the edge (no lookup, so it reads that face alone). 3D's front_face_out_normal_in().
    bool front_edge_out_normal_in(size_t a, size_t b, size_t band_fid, Vector2d& n) const;
    /// front_edge_field_angle() with the edge's band face given (front_edge_out_normal_in()). 3D's
    /// front_face_field_angle_in().
    double front_edge_field_angle_in(size_t a, size_t b, size_t band_fid) const;
    /**
     * @brief The field angle of offset edge (a, b): the angle, in degrees, between its unit
     * normal out of its band face (front_edge_out_normal()) and the field's unit normal at its
     * midpoint, the gradient of the distance d to the input (the edge's piece's exact integral,
     * ExactBandIntegral2D::distance()). 0 where the edge faces along the field, 180 where it faces
     * back toward the input. NaN where it cannot be measured: no band face, a degenerate edge, no
     * exact integral, or a zero gradient. 3D's front_face_field_angle(), its input envelope's
     * nearest primitive being 2D's integral's.
     */
    double front_edge_field_angle(size_t a, size_t b) const;
    /// Per-vertex 0/1: 1 at every fold's vertex (count_folds()). The debug frames' offset_foldover
    /// point field (write_vtu()); until the growth step of the 2D lock-step port also the
    /// end-of-run [foldover] warning, which the [turned edges] census replaced, as 3D's
    /// [turned faces]. Until the settle step it was 1 where the two offset edges at a vertex were
    /// within 30 degrees of each other (an outer angle over 330 through either side) -- the
    /// opposite test alone at a fixed angle, which called every ridge vertex a fold, as 3D's
    /// "real fold" did before 2026-11-02.
    std::vector<char> offset_surface_foldover_labels() const;
    /// An offset edge turned from the field: front_edge_field_angle() > foldover_angle; an
    /// unmeasurable edge is not one. A property of one edge, not a fold (fold_test(), which takes
    /// an unmeasurable edge as turned). 3D's front_face_turned().
    bool front_edge_turned(const size_t a, const size_t b) const
    {
        return front_edge_field_angle(a, b) > m_offset_params.foldover_angle;
    }
    /// The offset curve's edges turned from the field: over offset_surface_edges(), the edges
    /// whose front_edge_field_angle() exceeds foldover_angle, the unmeasurable ones (NaN, not
    /// counted as turned), and the worst angle with its edge's midpoint. The end-of-run
    /// [turned edges] warning and growth_mode "smart"'s test read it. 3D's turned_face_census().
    struct TurnedEdgeCensus
    {
        size_t n_turned = 0, n_edges = 0, n_unmeasured = 0;
        double worst = 0.;
        Vector2d worst_at = Vector2d::Zero();
    };
    TurnedEdgeCensus turned_edge_census() const;
    /// growth_mode "individual_op_reject": a collapse or a smoothing move that raises the number of
    /// offset edges turned from the field (front_edge_turned()) among the edges it changes is
    /// refused. In the loops only: not in the final pass (m_freeze_front, 3D's TetWild pass), nor
    /// before the march (the repulsion). As in 3D, whose rule also refuses a flip of the offset
    /// surface: 2D's swap never flips the front (the base refuses every tracked edge,
    /// swap_edge_before()), and an interior flip keeps each offset edge's band face on the same
    /// side of it, so its normal and field angle, and so cannot change which edges are turned.
    bool turned_reject_active() const
    {
        return m_offset_params.growth_mode == "individual_op_reject" && !m_freeze_front &&
               !m_repulsion_potential;
    }
    /// The live offset edges through vid turned from the field (front_edge_field_angle() >
    /// foldover_angle), as sorted vertex pairs, added to `out`. Reads vid's own face list and the
    /// faces in it only, as vertex_has_live_offset_edge() does, so the operation hooks may ask it.
    /// 3D's turned_front_faces_at().
    void turned_front_edges_at(size_t vid, std::set<std::array<size_t, 2>>& out) const;
    /// growth_mode "individual_op_reject": the turned edges through the collapse's two ends
    /// before it (turned_front_edges_at()), per thread; -1 for no rule. As in 3D.
    mutable wmtk::threading::enumerable_thread_specific<int> m_collapse_turned_before{-1};
    /// Operations refused for raising the turned edges (turned_reject_active()), per operation
    /// group; logged as [op reject]. As in 3D, which also counts flips (see turned_reject_active()).
    mutable std::atomic<int> m_turned_reject_collapse{0}, m_turned_reject_smooth{0};
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
     * IntersectionEnvelope, which must never be asked squared_distance (CompositeEnvelopes.hpp:
     * its BVH is null), so the members are walked individually instead of querying the composite.
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
    std::shared_ptr<Envelope> envelope_for_mask(uint64_t mask) const;

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
    std::shared_ptr<Envelope> surface_envelope_for_edge(
        const std::array<size_t, 2>& vids,
        const EdgeAttributes& /*attr*/) const override
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
        const std::shared_ptr<Envelope> base = containment_for(mask, all_offset);
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

    // smoothing_position_is_allowed(): the base's, the anchors' balls (hold_curves_and_points()),
    // as in 3D. Until 2026-11-11 this mesh anchored nothing and answered true. An isolated point
    // of a complex with faces arises only where two or more selected tags meet (the selection
    // labels an isolated simplex only where its face star is tag-heterogeneous), so the edges at
    // it are tag boundaries and the intersection of their tubes holds it
    // (smoothing_containment_envelope()); one of a complex without faces is anchored.

    /**
     * @brief The offset boundary is the one tracked surface with no envelope, in either role.
     *
     * It is the surface the optimization exists to move: a tube around wherever construction left
     * it would cap how far it can ever travel toward the level set. What holds it is the offset
     * term in the objective, not a container.
     *
     * The pull must be a real envelope, never a composite: this hook's consumers call the
     * non-virtual Envelope queries -- nearest_point and the ExactDistanceEnergy2D trio --
     * which on a composite would bind to the base's null BVH. So a junction vertex (several mask
     * bits) is pulled toward its most-violated member tube instead, one real envelope per
     * attempt, while the containment intersection below enforces the full constraint.
     */
    std::shared_ptr<Envelope> smoothing_energy_envelope(const size_t vid) const override
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
        std::shared_ptr<Envelope> best;
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
    std::shared_ptr<Envelope> smoothing_containment_envelope(const size_t vid) const
    {
        return containment_for(vertex_boundary_mask(vid), /*on_offset=*/false);
    }
    /// The smoother holds every tracked segment at the vertex to that one envelope, whatever the
    /// segment: the per-vertex answer above, not the base's per-segment one.
    std::shared_ptr<Envelope> smoothing_containment_envelope(
        const size_t vid,
        const std::array<size_t, 2>& /*edge*/,
        const EdgeAttributes& /*attr*/) const override
    {
        return smoothing_containment_envelope(vid);
    }

    /**
     * @brief Placement of a front vertex: the shared smoother with the offset's options. See
     * FrontSmooth2d.cpp.
     */
    bool smooth_front_vertex(const Tuple& t);
    /**
     * @brief A repulsion vertex in the passes and rounds before the march (repulsion_smoothing()):
     * the shared smoother at the front's options, against the AMIPS and O(v) once for every ring
     * face that carries it (repulsion_energy(), repulsion_cell_term()); the sum of tri_energy()
     * over its ring may not rise. See FrontSmooth2d.cpp. As in 3D.
     */
    bool smooth_repulsion_vertex(const Tuple& t);
    /**
     * @brief Every vertex off the front (and off the plastic medium): the shared smoother with
     * TriWild's options (TriOptimizerMesh::smooth_after()), except that it minimises
     * smoothing_extra_energy() -- the smoother adding no AMIPS term of its own -- and the engine's
     * AMIPS veto is replaced by the energy's: the sum of tri_energy() over the ring may not rise.
     * A vertex held on a region boundary is judged by it inside the engine's projected step
     * (smoothing_ring_energy()). See Optimize2d.cpp. As in 3D.
     */
    bool smooth_nonfront_vertex(const Tuple& t);
    /// The energy's AMIPS term over vid's one-ring, (w / s^3) sum area AMIPS^3
    /// (band_amips_weight(), area_amips3()), in the smoother's form (CubedAMIPSEnergy2D). As in
    /// 3D.
    std::shared_ptr<polysolve::nonlinear::Problem> amips3_energy(size_t vid) const;
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
     * energy_criterion()'s exit, refinement and ring measures, and the debug frames'
     * front_err_ratio and front_ring_ratio. Not part of the energy (tri_energy()) nor of the
     * smoother's objective, as 3D's face measure is not. -1 when unmeasurable (a sample whose
     * relative_residual() is not finite, which the euclidean field never has), +inf when the bar
     * is not positive.
     *
     * The (a, b) form reads the field of the chord (potential_for_edge()).
     */
    double edge_offset_term(size_t a, size_t b) const;
    /// `piece`: the chord's (place_piece()), whose bar the measure is in; -1, the run's. As in 3D.
    double edge_offset_term(
        const OffsetPotential2D& pot,
        const Vector2d& pa,
        const Vector2d& pb,
        int piece = -1) const;
    /// The field's outward unit direction at front vertex vid (zero where grad Phi vanishes).
    Vector2d front_vertex_normal(size_t vid) const;
    /// The objective of front vertex vid, as the smoother assembles it for a front vertex it
    /// places against the offset term (smoothing_extra_energy()): the AMIPS term over the one-ring
    /// (amips3_energy()), the rest-shape AMIPS of its plastic faces (rest_energy_for_vertex(),
    /// null without the plastic medium), and front_energy(). What
    /// front_vertex_normal_gradient() differentiates. As in 3D.
    std::shared_ptr<polysolve::nonlinear::Problem> front_objective(size_t vid) const;
    /// Whether vid is held on the input complex: on it (m_is_on_input) and held by an input
    /// envelope (a boundary mask, the one smoothing_containment_envelope() holds it to). There d =
    /// 0 and has a kink, so the band term is left out of vid's objective (smoothing_extra_energy())
    /// and a front vertex there cannot be placed at target_distance. A vertex held only by another
    /// region's boundary, the domain wall's among them, has neither: d is smooth there, so it
    /// carries the term and slides along that boundary to the level set. The 3D twin's rule, and
    /// its measurement (TopoOffsetTetMesh::vertex_is_held_on_input(): with every region boundary
    /// held, as 2D holds every one, the front could not settle where it meets another region).
    bool vertex_is_held_on_input(const size_t vid) const
    {
        return m_vertex_extra[vid].m_is_on_input && vertex_boundary_mask(vid) != 0;
    }
    /// Whether the smoother places vid against the offset term: a front vertex, outside the
    /// frozen-front final pass, not held on the input complex (vertex_is_held_on_input()). As in
    /// 3D. Until the smoother's step of the 2D port, a front vertex any envelope held carried no
    /// offset term.
    bool vertex_carries_offset_term(const size_t vid) const
    {
        return !m_freeze_front && m_offset_potential && m_vertex_extra[vid].m_is_on_offset &&
               !vertex_is_held_on_input(vid);
    }
    /// The AMIPS weight the rest-shape term uses at vid: offset_amips_weight for a vertex placed
    /// against the offset term, the engine's w_amips factor otherwise. As in 3D.
    double smoother_amips_weight(const size_t vid) const
    {
        if (vertex_carries_offset_term(vid)) return m_offset_params.offset_amips_weight;
        return m_params.w_amips > 0 ? m_s_amips * m_params.w_amips : 1.0;
    }
    /// THE smoothing objective at vid, for every vertex the shared smoother places: vid's part of
    /// the energy E, the sum of tri_energy() over vid's ring up to the terms that do not move with
    /// vid. The AMIPS term over the ring (amips3_energy()) for every vertex -- the smoother adds no
    /// AMIPS term of its own, every caller passing opts.w_amips 0 -- plus the band term of the
    /// ring's band faces (band_volume_energy_at(); through front_energy() for a front vertex the
    /// loop places) unless an input envelope holds vid on the input (vertex_is_held_on_input()),
    /// or, in the passes before the march, a repulsion vertex's term once per ring face that
    /// carries it (repulsion_energy(), repulsion_cell_term()). Under deform_others the rest-shape
    /// AMIPS of the deformable faces in the ring is added. As in 3D, where TetWild's rules give
    /// null (2D has none, see tri_energy()).
    std::shared_ptr<polysolve::nonlinear::Problem> smoothing_extra_energy(
        const size_t vid) const override
    {
        auto sum = std::make_shared<optimization::EnergySum>();
        sum->add_energy(amips3_energy(vid));
        // Before the march (repulsion_smoothing()) there is no front and nothing is plastic.
        if (is_repulsion_vertex(vid)) {
            sum->add_energy(repulsion_energy(vid));
            return sum;
        }
        if (vertex_carries_offset_term(vid)) {
            sum->add_energy(front_energy(vid));
        } else if (!vertex_is_held_on_input(vid)) {
            // E's band term moves with this vertex too -- except a vertex held on the input,
            // where d = 0 and has a kink: the Newton step along the input's normal is set by AMIPS
            // alone and lands far off, and the projected step discards the normal motion anyway.
            // 3D MEASURED (2026-10-09, cube, serial, 3 turns): with the term those solves took
            // 24.6 Newton iterations and ~20 line-search trials per iteration, without it 3.9 and
            // 1. E still guards the move: smooth_nonfront_vertex() vetoes on the ring's sum. 3D
            // leaves the term out under "exact" where it cancels over the ring
            // (band_term_cancels()), which 2D never finds under "exact" (see there); under
            // "exact_swept" the term is the search model, null where no boundary edge of B
            // passes through vid (band_volume_energy_at()), as in 3D.
            if (const auto bv = band_volume_energy_at(vid)) sum->add_energy(bv);
        }
        if (const auto rest = rest_energy_for_vertex(vid)) sum->add_energy(rest);
        return sum;
    }

    // ------- deform_others: other input regions deform instead of being envelope-held -------

    /// The released tags. Filled by release_deformable_regions(); empty = feature inactive.
    std::set<int64_t> m_deform_tags;
    /// The source tags (the selection's tags_involved), stored at release so the ops-only
    /// tube's edge classification applies the same never-freed rule the release did.
    std::set<int64_t> m_source_tags;
    /// The released boundaries' ops-only tube: an Envelope around the current deformed
    /// boundaries, consulted only by surface_envelope_for_edge() -- the dispatch every operation
    /// containment check goes through and no smoothing path does -- so operations preserve the
    /// current shape through remeshing while smoothing stays free to carry the object. Rebuilt
    /// lazily by released_envelope() when m_released_tube_dirty says a smoothing accept may have
    /// moved the boundary.
    mutable std::shared_ptr<Envelope> m_released_envelope;
    mutable std::atomic<bool> m_released_tube_dirty{false};
    mutable std::mutex m_released_mutex;
    /// The current released-boundary tube, rebuilt first if dirty. Null when nothing is
    /// released or no released-boundary segment exists.
    std::shared_ptr<Envelope> released_envelope() const;
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
    /// The offset term for a front vertex: the band term of its band faces
    /// (band_volume_energy_at()), in an EnergySum, empty when it has none. Defined in
    /// FrontSmooth2d.cpp. As in 3D.
    std::shared_ptr<polysolve::nonlinear::Problem> front_energy(size_t vid) const;
    /// The band term (band_cell_term()) of vid's band faces with vid moving (BandVolumeEnergy2D),
    /// null when vid has none. Defined in FrontSmooth2d.cpp. As in 3D.
    std::shared_ptr<polysolve::nonlinear::Problem> band_volume_energy_at(size_t vid) const;

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
     * Takes positions rather than a Tuple: a reader may pass a chord's ends in either order.
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
        /// every chord weighted equally. A vertex with any unmeasurable incident chord has no
        /// ring measure (n_rings_unmeasurable; the chord itself is already in n_unmeasurable).
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
    /**
     * @brief EXPERIMENTAL_settle_refinement, one turn, after energy_criterion(): the 2D twin of
     * TopoOffsetTetMesh::settle_refinement_turn(), front chords in place of offset faces. Settled
     * when the offset energy (offset_energy(), no AMIPS) fell by less than the bar energy
     * b = front_conv^2 L / (2 target_distance) since `e_prev` (the offset energy at the end of
     * the previous turn, turn 1: before its first group; set to the offset energy now), L the
     * front's length: the offset term's excess for a front lying front_conv beyond the level set,
     * per unit length int_0^front_conv s / delta ds (3D: A, the front's area). Only a settled turn
     * marks the front by EXPERIMENTAL_settle_mark and halves the corners it marks, when
     * `refine_now`; "vertex_split" lists chords for front_pocket_split() instead
     * (m_settle_split_edges). Logs the turn's [settle] line. Returns settled and nothing marked:
     * the loop's exit.
     *
     * The marks, one dimension down (a front chord f = (a, b) is 3D's front face; d the distance
     * to the input's segments and points, read through the face's piece's exact integral,
     * band_exact_for(place_piece()), 3D's envelope_for()):
     *   - "max": max(inside_f, outside_f) > front_conv, inside_f = delta - min over f of d
     *     (ExactBandIntegral2D::min_distance(), exact), outside_f = max over f of d - delta
     *     (ExactBandIntegral2D::max_distance()); under the other marks outside_f is the former
     *     bound, min over P of the max of d_P at f's ends (distance_bound()), logged only.
     *   - "rms": the mean over f of (d - delta)^2 (n . grad d), n f's normal out of the band, by
     *     the 2-point Gauss rule, over front_conv^2. Where 2D differs: 3D takes a triangle's
     *     3-edge-midpoint rule; both are the fewest-point rules with positive weights exact for
     *     quadratics on their simplex, so exact where d is affine along f.
     *   - "vertex": |d(v) - delta| > front_conv at a front vertex v; its own scalar is halved.
     *   - "vertex_edge": "vertex", and both ends of a chord whose ends are within front_conv and
     *     whose midpoint is not are halved, not holding the exit. 3D tests the edges of its front
     *     faces; 2D's front edges are its chords.
     *   - "vertex_split": "vertex", and such a chord (ends within, midpoint not) is listed to be
     *     split. 3D lists the worst-midpoint edge of a face whose corners are within but an edge
     *     midpoint or its centroid is not; a chord's one edge is itself and its centroid its
     *     midpoint, so 2D's "vertex_edge" and "vertex_split" test the same points and differ in
     *     the remedy alone, as 3D's do.
     *   - "stencil": a point of f's stencil (for_each_edge_sample()) at or beyond front_conv, or
     *     unmeasurable (edge_stencil_max_sq()). 3D reads its interior face stencil
     *     (EXPERIMENTAL_stencil_type); 2D's chord stencil holds the ends.
     * EXPERIMENTAL_gap_refinement as in 3D: a band face with a front corner whose corner mean
     * (ExactBandIntegral2D::min_mean_distance()) exceeds d at its centroid by front_conv has its
     * corners halved, and holds the exit.
     */
    bool settle_refinement_turn(int turn, double& e_prev, bool refine_now);
    /// The max over front chord e's stencil points (for_each_edge_sample()) of (d - delta)^2, as
    /// (delta r)^2, r the field's relative_residual() on e's piece (potential_for_edge()); +inf
    /// when a point is unmeasurable, 0 without a potential. EXPERIMENTAL_settle_mark "stencil"'s
    /// test; 3D's face_stencil_max_sq().
    double edge_stencil_max_sq(const std::array<size_t, 2>& e) const;

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
     * @brief Admits every collapse: switches off the engine's own AMIPS collapse rule (`q` the
     * AMIPS of a reshaped face, `ring_max` that of v1's ring), so the energy decides in its place
     * -- the sum of tri_energy() over the faces the collapse changes, before
     * (collapse_before_vertex()) against after (collapse_edge_after()). The AMIPS bound cannot
     * refuse early: the energy is a sum, and a face's AMIPS term can rise while the sum falls. As
     * 3D's loop rules; 2D has no TetWild rules (see tri_energy()).
     */
    bool collapse_quality_allowed(size_t, size_t, double, double) const override { return true; }

    /**
     * @brief Admits every swap: switches off the engine's own AMIPS swap rule (`after` the
     * largest AMIPS over the two new faces, `before` that of the old ones), so the energy decides
     * in its place, in swap_edge_after(). The AMIPS bound cannot refuse early, as for the
     * collapse. A 2D swap never flips the offset boundary (the engine refuses every tracked edge),
     * so `is_surface_flip` is always false. As 3D's loop rules.
     */
    bool swap_quality_allowed(double, double, bool) const override { return true; }

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
     * the loop and F in the final pass; led by m_frame_prefix (i during init_optimize, g<k> and
     * s0 / s<L>a<A> during growth_mode's stages). Same scheme as 3D.
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
     * @param E_tagged, E_tags, edge_tag_names, E_orientation: the tagged edges, as
     * InputData's F_tagged, F_tags, face_tag_names and F_orientation give them in 2D
     * @param point_vids: the input points' vertices (InputData::input_point_vids), each labelled
     * by the base's m_input_point_labels, which the caller sets before this with m_point_names
     */
    void init_from_image(
        const MatrixXd& V,
        const MatrixXi& F,
        const MatrixSi& F_tags,
        const MatrixXd& V_env,
        const MatrixXi& F_env,
        const std::vector<std::string>& tag_names,
        const std::string& curve_name = "",
        const MatrixXi& E_tagged = MatrixXi(),
        const MatrixSi& E_tags = MatrixSi(),
        const std::vector<std::string>& edge_tag_names = {},
        const MatrixSi& E_orientation = MatrixSi(),
        const std::vector<size_t>& point_vids = {});

    /**
     * @brief ensure ambient tag does not overlap any other tags in mesh.
     */
    bool ambient_assert();

    /// Label the input complex: the union of the complexes of Parameters::offsets, each entry's
    /// vertices carrying its index (VertexExtra2d::offset). Refuses a vertex two entries select.
    /// As in 3D.
    void label_input_complex();
    /// Label entry k's complex alone (label 1) as per its boolean expression (or single body
    /// mode), as label_input_complex() calls it on cleared labels; sets m_singlebody and
    /// m_single_tag for that entry. As in 3D.
    void label_offset(size_t k);

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
    /// Per row (a, b) of m_phi_E: the sign of orient2d(a, b, x) for x in the label-1 region it
    /// bounds (+1: the row's normal, b - a turned clockwise, points out of the region), 0 for an
    /// isolated segment of the complex. The 2D twin of TopoOffsetTetMesh::m_phi_F_inner; the
    /// exact integral's offset side reads it.
    std::vector<int8_t> m_phi_E_inner;

    /**
     * @brief The exact band integral over the input's segments and points (ExactBandIntegral2D,
     * on m_phi_V, m_phi_E, m_phi_P, m_phi_E_inner), built only where something reads it, as 3D's
     * TopoOffsetTetMesh::init_band_exact(): the band rules "exact" and "exact_swept" ("front_flux"
     * has no 2D port), "surface" with a label clean-up, settle_mark "max", DEBUG_output -- and,
     * where 2D differs, "corner_bound": its min over the input's primitives
     * (ExactBandIntegral2D::min_mean_distance()) is 3D's search on the input envelope, which 2D
     * does not have; and EXPERIMENTAL_settle_refinement under every settle_mark, whose tests 3D
     * runs on that envelope (settle_refinement_turn()). The label clean-up reads its distance()
     * and distance_bound(), the fold count its distance() with the gradient.
     * init_offset_potential() calls it. The 2D integral has neither 3D's value table nor its
     * bisector search (prepare_front_flux()).
     */
    void init_band_exact();
    std::shared_ptr<const ExactBandIntegral2D> m_band_exact;
    /// Several entries of Parameters::offsets: one exact integral per piece of the input complex,
    /// over its own primitives (init_band_exact()); empty otherwise. As in 3D.
    std::vector<std::shared_ptr<const ExactBandIntegral2D>> m_region_band_exact;
    /// The exact integral a place of piece r reads (place_piece()): its piece's, with several
    /// entries; m_band_exact, the whole complex's, otherwise. As in 3D.
    const std::shared_ptr<const ExactBandIntegral2D>& band_exact_for(const int r) const
    {
        return (r >= 0 && size_t(r) < m_region_band_exact.size()) ? m_region_band_exact[size_t(r)]
                                                                  : m_band_exact;
    }

    //// overriden splits/invariants
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
    bool invariants(const std::vector<Tuple>& tris) override;
    //// overriden splits/invariants

    /// Construction, start to finish, on the input mesh as given: the simplicial embedding,
    /// hold_curves_and_points(), repulsion_smoothing() when asked for, marching_tris() and the
    /// offset tagging. The optimization is optimize_offset(), which the driver calls afterwards.
    void construct_offset(const std::filesystem::path& output_file);
    /// The held curves' points and the input points, as triwild and simwild hold theirs: the 2D
    /// twin of TopoOffsetTetMesh::hold_curves_and_points(), one dimension down. A 2D held curve is
    /// codimension 1, a tracked region boundary held in its tags' tube from the load on
    /// (init_surfaces_and_boundaries(); the curve group among them), so what 3D tags and holds in
    /// tubes here -- its order-2 edges and feature curves, codimension 2 -- is in 2D a point,
    /// held by an anchor (the base's anchor_feature_points()): every vertex where the tracked
    /// curves end (one tracked segment) or three or more meet, at order2_envelope_ratio x
    /// envelope_size, 3D's order-2 edges; every input point (a point block,
    /// VertexExtra2d::point_inputs), selected or not, with its label, at feature_envelope_ratio x
    /// envelope_size, 3D's feature curves and points; and the isolated points of a complex
    /// without faces (a curve's or a point's), at that radius, as in 3D. After the simplicial
    /// embedding and before the repulsion and the march, so the front, which the march makes, is
    /// never among them. From here on the anchors carry the input points (m_points_held).
    void hold_curves_and_points();
    /// Set by hold_curves_and_points(): the anchors hold the input points, and the output's point
    /// block is written from them (io::collect_point_tags()). The 2D twin of
    /// TopoOffsetTetMesh::m_edge_tags_held, without its edge tags: 2D's (the edge block, on the
    /// edges' input sets) are codimension 1, the twin of 3D's face tags, tracked and held from the
    /// load on (init_from_image()).
    bool m_points_held = false;
    /// After the loop: what hold_curves_and_points() and the load set up, measured rather than
    /// assumed -- the 2D twin of TopoOffsetTetMesh::audit_input_curves(). The held curves (the
    /// tracked region-class edges, the curve group's among them), those of them outside their
    /// tube; for a complex without faces (a curve or point offset's), its vertices and those of
    /// them on no held curve and no anchor (3D: its edges without the feature tag); and anchors
    /// with no vertex in their ball (the base's feature_retention()). A warning iff there is a
    /// defect.
    void audit_input_curves() const;

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
    /// `result`: the final mesh, its annotations trimmed; otherwise a debug frame, every field.
    void write_vtu(const std::string& path, MeshMap* output = nullptr, bool result = false);
    // void write_msh(const std::string& file);
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
