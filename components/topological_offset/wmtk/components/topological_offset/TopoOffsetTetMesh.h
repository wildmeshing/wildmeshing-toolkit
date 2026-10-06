#pragma once

#include <array>
#include <atomic>
#include <cmath>
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
    bool m_is_on_input = false; // on the input complex
    bool m_is_on_offset = false; // on the offset surface itself
    bool m_is_on_region = false; // on some OTHER tag region's boundary

    /// Which split pass created this vertex, from wmtk::TetOptimizerMesh::m_op_epoch; 0 means not
    /// created by an optimization split. Read only by the needle diagnostics' per-vertex lines.
    /// Assigned at each split, never OR'd -- a recycled slot carries a dead vertex's epoch.
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
     * oriented order. Stamped for every plastic cell at optimize_offset() and re-stamped by the
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
        SphereTrace = 1, // marching_tets (construction_mode): sphere tracing along the edge to
                         // d(x) = m_construction_distance, midpoint when the trace leaves the
                         // edge
        Optimization = 5 // the optimization phase; the shared engine places the vertex
    };

public:
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
     * construct_offset() runs, so this holds the original geometry however the elements
     * representing the complex are later remeshed. Rebuilding from the live mesh would redefine
     * the offset distance in terms of a surface the optimizer had just moved.
     *
     * Containment is not its job -- m_envelope holds the complex's boundary in place.
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
     * @brief THE ENVELOPES. One, m_envelope (the base's pointer): a SampleEnvelope of half-width
     * envelope_size around every HELD face (face_is_held()), built once by build_envelopes()
     * before anything moves -- after the simplicial embedding, before the repulsion and the
     * march -- and never rebuilt: what it holds does not move. The offset surface is held by
     * nothing in the loop; only the frozen-front final pass holds it, to m_offset_envelope.
     *
     * Which faces are held, asked live of the mesh (the operations carry the cell tags and
     * labels it reads), never stored:
     * - a domain-wall face (no opposite tet), always;
     * - deform_others false: every other tracked face that is not the offset surface, i.e.
     *   every region boundary, the input complex's included;
     * - deform_others true: the input complex's boundary only (face_is_complex_boundary()).
     *   The other regions' boundaries stay tracked faces -- the operations preserve their
     *   topology -- but no envelope holds them: they deform with the plastic medium.
     * One envelope for all of them, as TetWild's: where two held surfaces meet, a vertex is
     * kept within eps of their union, not pinned to the junction curve.
     */
    bool face_is_held(const Tuple& f) const;
    /// A vertex is held when one of its tracked faces is (face_is_held()): the smoother then
    /// pulls it to and contains it in m_envelope. Asked of the mesh, like face_is_held().
    bool vertex_is_held(size_t vid) const;
    /// Whether this face lies on the boundary of the input complex: exactly one incident tet
    /// carries label 1, or the face itself does while neither tet does (a sheet or face piece).
    bool face_is_complex_boundary(const Tuple& f) const;
    /// Build m_envelope from the held faces. Called once, from construct_offset(), after the
    /// simplicial embedding: the complex has to be labelled for face_is_held(), and it has to
    /// precede the march, whose band replaces the tags of the cells it covers.
    void build_envelopes();

    /// The final pass: front vertices are not smoothed (see smooth_before()).
    bool m_freeze_front = false;

    /**
     * @brief The tube the offset surface may not leave during the frozen-front final pass, of
     * half-width offset_envelope, built by build_offset_envelope() from the surface as the loop
     * left it, just before that pass. Null until then: the loop holds the offset surface to no
     * envelope. The surface is a closed manifold by then (offset_is_manifold() is the driver's
     * check), so it is one triangle set with no junctions: a face is either on it or not, and
     * surface_envelope_for_face() answers an offset face with this envelope alone.
     */
    std::shared_ptr<SampleEnvelope> m_offset_envelope;
    void build_offset_envelope();

    /// Hard error if any vertex is on both the input complex and the offset surface -- a state
    /// no placement satisfies. Called at construction.
    void check_no_vertex_on_both_surfaces(const char* when) const;

    /// TetWild's loop, the front placed inside its smoothing passes. `final_stage` false is the
    /// init_optimize loop: it returns on convergence without the
    /// frozen-front final pass and without deciding the verdict. `refine` false skips the
    /// halving. `label` names the loop in its log.
    void optimize_offset_loop(
        bool final_stage = true,
        bool refine = true,
        const std::string& label = std::string());

    /// init_optimize: optimize_offset() opens with a stencil_order loop
    /// without refinement. Set by marching_tets(), only when the target is beyond the maximum
    /// marchable distance.
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
    // m_envelope itself lives on the base, which is what checks tracked-surface triangles
    // against it.
    double m_envelope_eps = -1;

    /**
     * @brief SurfaceTagAttributes::m_surface_class: which of the two tracked surfaces a face
     * belongs to. Same scheme as 2D.
     *
     * OFFSET is the surface the optimization places at target_distance. Everything else -- the
     * input complex, another body's boundary, the domain wall -- keeps the primary class 0 and
     * is envelope-checked by the shared operations exactly as in tetwild and simwild, when it is
     * held (face_is_held()).
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
     * @brief THE per-tet energy, read from the mesh as it is -- labels, neighbours and positions,
     * nothing hypothetical.
     *
     *     E(t) = A(t)^3 + [t is band] * sum over the live front faces f of t of O(f)
     *     O(f) = (1/N) sum_i (relative_residual(q_i) / front_conv_frac())^2
     *
     * A(t)^3 is the base's TetOptimizerMesh::get_quality(), the AMIPS^3 the engine stores as the
     * cell quality; its MAX_ENERGY (unscoreable) passes through unchanged. q_i are the N points
     * of f's stencil_order stencil (for_each_face_sample()) on the field of f's band cell
     * (potential_for_face()), f's corners sorted so that a face's term does not depend on which
     * cell or which operation reads it. O is face_offset_term(), the face measure
     * the squared face measure: the error IN UNITS OF THE TOLERANCE front_conv, so O = 1 exactly
     * at the bar. No weights anywhere: no area, no w_amips, no valence normalisation. With no
     * field yet (m_offset_potential null) E = A^3.
     *
     * A face whose term is unmeasurable (a stencil point where relative_residual() is not
     * finite; the euclidean field has none) makes the cell MAX_ENERGY, the engine's own
     * "unscoreable", so the rules treat it as they treat a degenerate cell: no operation may make
     * a measurable neighbourhood unmeasurable. The ops guard this replaced read an unmeasurable
     * face the same way (+inf).
     *
     * THE BAND SIDE CARRIES THE FACE: only a band cell (cell_is_offset_band()) adds terms, and a
     * live front face has exactly one band side (face_is_offset_surface_live()), so every front
     * face is counted once; a band cell with two front faces carries both. The cell across is
     * background -- under deform_others the plastic medium, with a rest shape of its own -- and
     * carries none.
     *
     * WHERE IT IS COMPARED: in the application's after-hooks, on the real mesh, each against the
     * max over the same cells as its before-hook cached them -- the collapse in
     * collapse_after_connectivity(), the swaps in swap_after_cells(), the front smoother in
     * smooth_front_vertex() -- and, for the swaps, also where the engine picks and gates
     * the cells a swap would make (CANDIDATE CELLS below). Not in the engine's own quality rules:
     * the engine scores collapse and swap candidates BEFORE they exist, by vertex ids
     * (TetOptimizerMesh::collapse_edge_before(), TetMesh's 4-4 / 5-6 case search, the face swap),
     * through its AMIPS^3 get_quality(), which is not virtual, and an energy that needs a cell's
     * label and the cell across each of its faces cannot be read off a cell that has neither yet.
     * So the engine's stored cell quality stays AMIPS^3 and every engine diagnostic reads it
     * unchanged; the application switches off the engine's collapse and swap quality rules
     * (collapse_quality_allowed(), swap_quality_allowed()), keeps its AMIPS veto off for front
     * vertices, overrides the swaps' candidate scoring (swap_edge_44_energy(),
     * swap_edge_56_energy(), swap_face_before()), and applies this energy in their place. The
     * front smoother minimises the same two parts at 1:1: AMIPS at weight 1 and StencilEnergy3D
     * at offset_term_weight() -- exactly this sum for the euclidean field.
     *
     * CANDIDATE CELLS: the 4-4 and 5-6 case searches (TetMesh::swap_edge_44() / ::swap_edge_56())
     * and the face swap's gate (swap_face_before()) score cells that do not exist yet and take a
     * candidate only when it scores strictly below the current cells. The current cells are real
     * and are read here (op_case 0). A candidate has no slot, so it cannot be looked up; it is
     * scored by candidate_energy() from the swap's record (SwapRecord), which
     * swap_before_interior() / swap_before_surface() fill before the search runs. A swap moves no
     * vertex, so a candidate's AMIPS^3 comes from its four positions and every front term it can
     * carry is a constant of the operation: the record holds each face a new cell can carry, its
     * term, which new cells are its band side, and the vertices of the replaced cells, outside
     * which a scored cell is a defect. The after-hook swap_after_cells() then applies the same
     * rule on the real, labelled cells and stays THE rule; under perform_sanity_checks it compares
     * the energy the search or the gate scored for the committed configuration with the max of
     * this energy it reads, which ties the two paths together. Until 2026-09-28 the search and the
     * gate scored AMIPS^3 alone, so a 4-4, 5-6 or face swap that lowers the energy while raising
     * AMIPS was discarded before the rule saw it -- the flips the energy exists to admit: at the
     * cube 1e-3, 93.5% of the valence-4 surface edges whose flip would cut the sag raised max
     * AMIPS (see swap_after_cells()).
     */
    double tet_energy(size_t tid) const;
    /// tet_energy() with the cell's AMIPS^3 given, as the shared smoother has just computed it.
    double tet_energy(size_t tid, double amips3) const;
    /// The shared smoother's per-cell comparison -- its projected step for a vertex held on a
    /// surface, and its quality veto -- is the per-tet energy, so every smoothing test judges a
    /// move by the energy the smoother minimises (smoothing_extra_energy()).
    double smoothing_cell_energy(const size_t tid, const double quality) const override
    {
        return tet_energy(tid, quality);
    }
    /// The AMIPS part of the energy: offset_amips_weight times AMIPS^3, the MAX_ENERGY sentinel
    /// (unscoreable) passed through unscaled so that it stays the largest value anywhere. Every
    /// place that turns AMIPS^3 into energy reads it through here -- tet_energy(),
    /// candidate_energy(), and the energy rules' early halves on the engine's AMIPS^3 bound.
    double weighted_amips(const double amips3) const
    {
        return amips3 >= MAX_ENERGY ? amips3 : m_offset_params.offset_amips_weight * amips3;
    }
    /// Max of tet_energy() over `tids` (0 for none): the number every rule above compares.
    double max_tet_energy(const std::vector<size_t>& tids) const;
    /// tet_energy() of a cell the swap in flight may create, which does not exist yet: AMIPS^3
    /// from the four positions (MAX_ENERGY passes through) plus the terms of the faces of `vids`
    /// the swap's record (SwapRecord) says this cell carries. A cell not made of the record's
    /// vertices, or asked with no record, is a defect and throws. See CANDIDATE CELLS above.
    double candidate_energy(const std::array<size_t, 4>& vids) const;
    /// 1 / front_conv_frac()^2: the factor that turns a squared relative error into a squared
    /// error in units of the tolerance. The weight of the front smoother's offset terms, and the
    /// scale of face_offset_term().
    double offset_term_weight() const
    {
        const double f = m_offset_params.front_conv_frac();
        return 1. / (f * f);
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
    /// Every live offset-surface face as a sorted vertex triple.
    std::vector<std::array<size_t, 3>> offset_surface_faces() const;
    /// The live offset-surface faces incident to vid.
    std::vector<Tuple> offset_surface_faces_live_at(size_t vid) const;
    /// Whether ANY live offset-surface face is incident to vid. The same question
    /// offset_surface_faces_live_at() answers, without building the list: this one runs in the
    /// operation hooks, where the list would be allocated and thrown away.
    bool vertex_has_live_offset_face(size_t vid) const;
    /**
     * @brief Re-derive m_is_on_offset for one vertex from the cell labels, exactly.
     *
     * THE definition of the flag, and the only thing that writes it after
     * label_offset_boundary(): a vertex is on the offset surface iff some incident face has the
     * band on one side and a non-complex cell on the other. Called from the three hooks where an
     * operation can change the answer -- see the note above m_collapse_edge_link.
     *
     * Reads labels, never the flag it is writing and never the cached face class, so a wrong
     * value cannot propagate and any vertex an operation touches is corrected whatever it carried
     * before.
     *
     * It deliberately does NOT touch m_vertex_attribute[vid].m_is_on_surface, which is the base's
     * union over every tracked surface (input, region, offset); clearing that from here would
     * unhold a vertex that is still on a region boundary. See the CLAUDE.md note.
     */
    void refresh_offset_membership(size_t vid);
    /// perform_sanity_checks: how many vertices carry m_is_on_offset without a live offset face,
    /// and how many are the other way round. Whole-mesh, O(V x ring); zero is the invariant.
    std::pair<size_t, size_t> offset_membership_mismatches() const;
    /// Log offset_membership_mismatches() and throw when it is not {0, 0}. perform_sanity_checks
    /// only.
    void check_offset_membership(const char* when) const;

    /**
     * @brief Faces that offset_surface_faces_live_at() asked for and the connectivity did not
     * have.
     *
     * It walks a vertex's one-ring of tets and then steps across each face to the tet on the
     * other side, so it reads two and three hops out from the seed. (vertex_has_live_offset_face()
     * walked the same way until 2026-09-30; it now pairs the faces within the vertex's own tets
     * and reads nothing the lock does not hold.) The collapse and swap passes
     * guarantee only `{v1, v2} u N(v1) u N(v2)` -- see "Ring lockers -- NOT balls" in TetMesh.h --
     * so at num_threads > 0 a neighbouring thread can be shrinking a vertex fan these walks are
     * reading, and the face lookup misses. A miss is taken as "not a live offset face", which is
     * what TetMesh.h's try_tuple_from_face doc calls the legitimate answer.
     *
     * Counted because a miss means the m_is_on_offset just written was derived from a stale read
     * and may be wrong. Zero is the expected value. Non-zero says the walks are racing the pass,
     * and the remedy is to widen the collapse pass's lock (collapse_all_edges_impl's
     * exact_ball_lock), which is shared code and not an offsets-side change.
     *
     * History: before 2026-09-18 neither walk checked for a miss. The asserting tuple_from_face
     * returns a default Tuple under NDEBUG, whose m_global_tid is size_t(-1), and
     * switch_tetrahedron() indexed m_tet_connectivity with it -- one element below the vector's
     * base. That was the SIGSEGV on the cube at target_distance_rel 1e-3.
     */
    mutable std::atomic<long long> m_offset_face_lookup_misses{0};
    /// face_is_offset_surface_live() calls handed an invalid Tuple (m_global_tid == size_t(-1)).
    /// Defence in depth behind the two walks above: with both of them checking their lookups this
    /// should stay 0, and a future caller that forgets gets `false` instead of a wild read.
    mutable std::atomic<long long> m_offset_face_invalid_tuple{0};
    /// Log the two counters above, run totals, and only when either is non-zero -- a clean run
    /// prints nothing. Not gated on perform_sanity_checks: these are free unless they fire.
    void report_offset_face_lookup_misses(const char* when) const;

    /**
     * @brief Per-vertex 0/1: is this vertex an endpoint of a COLLAPSED (folded-over) offset
     * surface edge? Debug-frame diagnostic; see write_vtu(). Costs one pass over the live
     * offset faces, no field evaluation.
     *
     * An offset-surface edge carries two live offset faces. Measured through either side, the
     * angle between them is 180 degrees where the surface is flat and 360 where the two faces
     * lie on top of each other with that side pinched to nothing. Over FOLDOVER_OUTER_ANGLE_DEG
     * through EITHER side is the fold, and every such edge's two endpoints get 1.
     *
     * Which side is pinched is deliberately not determined: on the cube the measured folds
     * pinch the BACKGROUND, not the band, so a test written around a pinched band found none of
     * them. Since the two sides sum to 360, the test is simply that the unsigned angle is under
     * 360 minus the threshold. Vertices of an edge that does not carry exactly two live offset
     * faces are left 0, as are degenerate faces: this is a diagnostic, and a number it cannot
     * measure is not a fold.
     *
     * The 2D twin is TopoOffsetTriMesh::offset_surface_foldover_labels(), which asks the same
     * question of a curve vertex's two incident offset edges.
     */
    std::vector<char> offset_surface_foldover_labels() const;

    /// {max_dist_err, avg_dist_err, max_phi_residual, avg_phi_residual, max_grad, avg_grad,
    /// max_grad_at_vertex, max_grad_in_face}. One entry for the whole run, as in 2D.
    std::vector<std::array<double, 8>> optimization_metrics;
    /// The turn the run is in, 1-based; 0 before the loop starts. Read only by
    /// write_optimization_debug_output(), to tag each frame with the turn it belongs to.
    int m_round = 0;
    /// Monotonic frame counter for the debug timeline.
    mutable size_t m_debug_seq = 0;
    /// DEBUG_output: the label of each debug frame, indexed by its sequence number, and, per
    /// companion suffix, the frame indices that actually produced one. Both exist only to write
    /// the ParaView collections -- see write_debug_pvd(). Same in 2D.
    mutable std::vector<std::string> m_debug_frame_labels;
    mutable std::map<std::string, std::vector<size_t>> m_debug_pvd_series;
    /// DEBUG_output: rewrite <output>{_main,_off,_surf,_edge,_front}.pvd, a ParaView time
    /// series over the debug frames. Needed because ParaView only groups a file series when the
    /// index is immediately before the extension, which is false for every companion
    /// (<output>_NNNNN_off.vtu). Called after every frame, so a killed run still opens.
    void write_debug_pvd() const;
    /// Pass index within the current turn, and the (turn, loop-or-final-pass tag) it belongs to --
    /// when those change the index restarts. All three exist only to name frames.
    mutable int m_debug_pass = 0;
    mutable int m_debug_last_round = -1;
    mutable char m_debug_last_tag = '?';
    mutable std::string m_debug_last_prefix = "?";
    /// The run's verdict: the front resolved (EnergyCriterion::converged(): every offset face's
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
    /// Operations refused because they would have left an offset-surface face over tolerance.
    std::atomic<int> iter_cnt_collapse_offset_reject{0};
    std::atomic<int> iter_cnt_swap_offset_reject{0};
    /// The energy rules (see tet_energy()): collapses refused for raising the max energy over
    /// the survivor's ring, swaps for not strictly lowering it over the cells they make.
    mutable std::atomic<int> iter_cnt_collapse_energy_reject{0}; // both halves count here
    mutable std::atomic<int> iter_cnt_swap_energy_reject{0};
    /// perform_sanity_checks: swaps whose cells the case search or the face gate scored before
    /// they existed (SwapRecord::scored_energy), compared in swap_after_cells() against the same
    /// cells read on the mesh, and those that read differently (a warning each, the first 8 per
    /// turn). Reported and reset once a turn as [swap scoring].
    std::atomic<long long> m_swap_scoring_checked{0};
    std::atomic<long long> m_swap_scoring_mismatch{0};

    /**
     * @brief [flip funnel]: of the flips of the offset surface, how many survive each stage.
     * Reset per turn and reported next to [swap reject].
     *
     * [swap reject] counts every refusal of every surface flip, most of which SHOULD be refused.
     * This follows only the flips of the offset surface -- both replaced faces offset surface --
     * through the base's case search and then swap_after_cells(), which captures the new cells'
     * sides and then applies the energy rule (see tet_energy()).
     *
     * Reading it. `offset-surface flips` is counted in swap_before_surface(), which is the app's
     * first sight of a candidate; anything the base turned down earlier (valence, bbox,
     * connectivity) never reaches it and is in [swap reject] instead. For a 4-4 or 5-6, what does
     * not reach swap_after_cells() is lost in the base's case search, which scores on the per-tet
     * energy (swap_edge_44_energy()): no retetrahedralization that makes the (c,d) diagonal was
     * found, or none scored strictly below the current cells (the `cases` split). A 3-2 reaches
     * swap_after_cells() unless a new cell inverts: swap_quality_allowed() admits every swap.
     * after_cells - energy is swap_after_cells() refusing on the side/label capture, energy -
     * committed the energy rule refusing. What the envelope check then refuses is past this hook
     * and shows as after_envelope in [swap reject].
     */
    mutable std::atomic<long long> funnel_offered{0};
    /// offered, split by swap kind: [0] = 3-2, [1] = 4-4, [2] = 5-6.
    mutable std::array<std::atomic<long long>, 3> funnel_kind{};
    /// of the 4-4 and 5-6 ones, those for which at least one case survived accept_case and was
    /// scored. offered(4-4 + 5-6) - this is exactly what flip_wrong_case threw away.
    mutable std::atomic<long long> funnel_cases{0};
    /// The scored CASES of those flips, by what the case search's energy (the per-tet energy)
    /// said about them against the current cells (SwapSurfaceSides::case0_energy). An inverted
    /// case is one whose retetrahedralization inverts a cell -- swap_edge_*_energy returns
    /// double::max() for that -- which is a geometric refusal, not a quality one. Only a better
    /// case passes the base's `energy < min_energy` test, so case_better is what can still become
    /// a swap.
    mutable std::atomic<long long> funnel_case_inverted{0};
    mutable std::atomic<long long> funnel_case_not_better{0};
    mutable std::atomic<long long> funnel_case_better{0};
    mutable std::atomic<long long> funnel_after_cells{0};
    mutable std::atomic<long long> funnel_energy{0};
    mutable std::atomic<long long> funnel_committed{0};
    std::string flip_funnel_report() const;
    void flip_funnel_reset();
    /// Splits of an offset-surface edge: offered, accepted.
    std::atomic<int> iter_cnt_split_offset_before{0};
    std::atomic<int> iter_cnt_split_offset{0};
    /// Longest-edge order in the optimization split (see split_edge_before()), counted per turn:
    /// splits that waited for a strictly longer edge of an incident tet that was over the split
    /// gate, and committed splits whose edge was not the longest edge of every incident tet.
    std::atomic<long> m_split_order_waits{0};
    mutable std::atomic<long> m_split_off_longest{0};
    /// Whether the split running on this thread is off the longest edge of an incident tet. Set
    /// by split_edge_before(), counted by op_event() when that split commits.
    static bool& split_off_longest()
    {
        static thread_local bool off = false;
        return off;
    }
    void op_event(OpKind k, OpEvent e) const override;
    /// The shared split pass's gate: TetOptimizerMesh::split_all_edges's is_weight_up_to_date
    /// without its staleness test.
    bool split_edge_is_due(const Tuple& e) const;

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
     */
    bool edge_split_sphere_trace(
        const Vector3d& p_in,
        const Vector3d& p_out,
        Vector3d& p_new,
        size_t& steps) const;
    /// marching_tets() tallies for the construction log: edges placed on the level set / at the
    /// midpoint because the trace left the edge, and the trace steps (total, max). Reset at the
    /// start of marching_tets().
    size_t m_marching_root_splits = 0, m_marching_midpoint_splits = 0;
    size_t m_marching_trace_steps = 0, m_marching_trace_steps_max = 0;
    /// The distance the marching's sphere trace aims for: target_distance, or half the maximum
    /// marchable distance under construction_mode "max_marchable_fallback" (see the marching).
    double m_construction_distance = 0.;

    /**
     * @brief Reject any collapse that violates the substructure link condition, remember the
     * survivor's sizing for sizing_collapse_min = false, and cache the energy rule's before-half.
     *
     * The base applies the link condition only when both endpoints already sit on a tracked
     * surface or the bbox; the offset region is a thin shell, so a collapse with one endpoint in
     * the interior can still pinch its two sides together. The offset asks unconditionally.
     */
    bool collapse_edge_before(const Tuple& t) override;
    /// The coarsening bar, the sizing restore and the rest re-stamp, after the base accepted.
    bool collapse_edge_after(const Tuple& t) override;
    bool collapse_before_vertex(size_t v1, size_t v2, double edge_length) override;
    /// The collapse's energy rule (see tet_energy() and the definition), and in coarsening the
    /// absolute offset bar.
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
     * surface-flip refusal (class match, same boundary). The geometric half is the shared swap's
     * containment check. Both also cache the energy rule's before-half (SwapEnergyBefore) and
     * fill the record candidate cells are scored from (SwapRecord). See Optimize3d.cpp.
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
     * tags its faces m_is_surface_fs and face_is_held() always holds them, so refinement,
     * coarsening, flips and smoothing are governed by the same containment, merge rules and link
     * conditions that govern the input complex.
     */
    bool vertex_is_on_domain_boundary(const size_t vid) const
    {
        return !m_vertex_attribute[vid].on_bbox_faces.empty();
    }

    /**
     * @brief Classify every region boundary and tag the domain wall -- once, from the input mesh,
     * before offset construction runs. A region boundary is a face whose two incident tets carry
     * different tag sets, a sheet-group face, or a face with one incident tet (the domain wall);
     * each becomes a tracked face (m_is_surface_fs). Which of them an envelope holds is
     * face_is_held()'s, and the envelope is build_envelopes()'s.
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

    /// How the solves this class makes itself ended, per smoothing pass, beside the base's
    /// m_newton (the background, through TetOptimizerMesh::smooth_after()). Front: every front
    /// placement -- the 1-D solve along the field normal and the 3-D
    /// solve it falls back to. Plastic: the rest-shape solve of smooth_plastic_vertex(). Logged
    /// and reset by log_smoothing_pass_accounting().
    optimization::NewtonCounters m_newton_front;
    /**
     * @brief DEBUG_output only: every front solve since the last debug frame, so the frames
     * show per vertex what m_newton_front counts per pass (frame fields front_newton_iters and
     * front_newton_status, see write_vtu()).
     *
     * Appended by smooth_front_vertex(), emptied by write_debug_frame() once the frame is
     * written. The engine writes a frame after every pass and smooths a vertex at most once per
     * pass, so a smoothing pass's frame shows each front vertex's solve of that pass and an
     * operation pass's frame shows none. Not in m_vertex_extra: a refused move rolls the vertex
     * attributes back (TetMesh::smooth_vertex()), and the refused solves are the ones to see.
     * Added 2026-10-01 for Thingi10K 100026 at target_distance_rel 5e-3, where front solves end
     * at the 10-iteration cap ever more often inside the model's two slots and the per-pass
     * counts cannot say where.
     */
    struct FrontSolveRecord
    {
        size_t vid;
        int iterations;
        int status; ///< polysolve's stop status + 1: NewtonCounters::status_name()'s numbering
    };
    std::vector<FrontSolveRecord> m_front_solve_log;
    std::mutex m_front_solve_log_mutex;
    /**
     * @brief The Newton stopping rule every smoother of this component runs with, set on the
     * thread's shared solver (the engine's m_solver slot) by smoothing_solver() at the start of
     * every visit -- front, plastic-medium and interior alike, so the settings do not depend on
     * which path first ran on a thread.
     *
     * Relative gradient tolerance 1e-6, beside the engine's absolute 1e-10 (which stays). The
     * front objective is in tolerance units, 1e4 times the old front objective, and its
     * gradient's round-off floor sits at 1e-11..1e-9: measured 2026-09-28 on the cube (target
     * 1e-2, tolerance 1e-4), 65% of front solves ran to the 10-iteration cap converged to
     * machine precision (|grad|/|grad_0| at 1e-14..1e-11), unable to meet the absolute 1e-10.
     * With the relative rule: mean 2.8 iterations (exact Hessian) instead of 9.7, 98% stopped on
     * it. Interior solves, which met the absolute rule in 3.8 iterations, stop a little earlier.
     */
    static constexpr double kSmoothRelGradNormTol = 1e-6;
    /// The thread's shared solver, created with the engine's parameters if needed, with
    /// kSmoothRelGradNormTol applied. Every smoothing path of this component takes it from here.
    polysolve::nonlinear::Solver& smoothing_solver();
    /// The front veto (smooth_front_vertex(), solve_3d): moves whose Newton solve
    /// succeeded and reached the veto, and how many it refused for raising the ring's max
    /// tet_energy. Reported and reset per pass beside the Newton counters. Kept apart from
    /// m_smooth_rejects.quality, which the engine's own veto on interior vertices also counts.
    /// A refusal here is expected, not a defect: the solve lowers the SUM of the ring's energy
    /// and the veto bounds its MAX, and a step can lower the sum while raising the worst cell.
    std::atomic<size_t> m_front_veto_asked{0}, m_front_veto_fired{0};
    /// The repulsion passes and rounds (repulsion_smoothing()), set only while they run: the
    /// Euclidean field at level 2 x target_distance. Null otherwise, so every other path is
    /// untouched.
    std::shared_ptr<const OffsetPotential3D> m_repulsion_potential;
    /// Per repulsion pass or round, as m_newton_front and the front veto counters are per loop
    /// pass.
    optimization::NewtonCounters m_newton_repulsion;
    std::atomic<size_t> m_repulsion_veto_asked{0}, m_repulsion_veto_fired{0};
    /// The edges marching_tets() splits: exactly one end in the input complex (label != 0).
    bool is_marched_edge(const size_t a, const size_t b) const
    {
        return (m_vertex_extra[a].label == 0) != (m_vertex_extra[b].label == 0);
    }
    /// While the repulsion runs: an outer end of a marched edge -- off the complex, with a
    /// neighbour on it -- that no envelope holds. Asked of the mesh every time, so it follows the
    /// repulsion rounds' operations.
    bool is_repulsion_vertex(const size_t vid) const
    {
        if (!m_repulsion_potential || m_vertex_extra[vid].label != 0 || vertex_is_held(vid)) {
            return false;
        }
        for (const size_t u : get_one_ring_vids_for_vertex(vid)) {
            if (m_vertex_extra[u].label != 0) return true;
        }
        return false;
    }
    /// O(v) = (max(0, 2 delta - d(v)) / front_conv)^2: the Euclidean residual against level
    /// 2 delta is (d - 2 delta) / (2 delta), so the weight (2 delta / front_conv)^2 =
    /// 4 offset_term_weight() puts it in units of the tolerance, as the front's terms are. One
    /// object for the whole repulsion (value() changes nothing in it, so threads share it); set
    /// with m_repulsion_potential.
    std::shared_ptr<OffsetEnergy3D> m_repulsion_term;
    double repulsion_term_at(const size_t vid) const
    {
        Eigen::VectorXd x = m_vertex_attribute[vid].m_posf;
        return m_repulsion_term->value(x);
    }
    /// THE REPULSION PART OF THE PER-TET ENERGY, before the march (tet_energy(), candidate_energy()):
    /// a cell carries O(v) of each of its corners v that is an outer end within it -- off the
    /// complex, held by no envelope, in a cell that has a complex vertex, i.e. joined to the
    /// complex by a marched edge of this cell. These cells are the band the march will make, so
    /// this is the counterpart of a band cell carrying its front face's term. Reads the cell's
    /// own four vertices only. 0 while the repulsion does not run.
    double repulsion_cell_term(const std::array<size_t, 4>& vids) const
    {
        if (!m_repulsion_potential) return 0.;
        bool has_complex = false;
        for (const size_t v : vids) has_complex = has_complex || m_vertex_extra[v].label != 0;
        if (!has_complex) return 0.;
        double e = 0.;
        for (const size_t v : vids) {
            if (m_vertex_extra[v].label == 0 && !vertex_is_held(v)) {
                e += repulsion_term_at(v);
            }
        }
        return e;
    }
    /// How many cells of vid's ring carry its term (repulsion_cell_term()): the smoother's
    /// objective is the ring's sum of the per-tet energy, so it charges O(v) that many times.
    size_t repulsion_cells_at(const size_t vid) const
    {
        size_t n = 0;
        for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
            for (const size_t u : oriented_tet_vids(tid)) {
                if (m_vertex_extra[u].label != 0) {
                    ++n;
                    break;
                }
            }
        }
        return n;
    }
    /// O(v) for vid's smoothing objective, charged once per carrying cell.
    std::shared_ptr<OffsetEnergy3D> repulsion_energy(const size_t vid) const
    {
        return std::make_shared<OffsetEnergy3D>(
            m_repulsion_potential,
            double(repulsion_cells_at(vid)) * 4. * offset_term_weight(),
            true,
            true,
            /*one_sided=*/true);
    }
    /// repulsion_rounds: whether the complex is still simplicially embedded around the cells
    /// `tids` an operation just made -- no cell off the complex has complex vertices spanning a
    /// simplex off the complex (tet_is_simp_emb()'s test, decided from the cells' labels, which
    /// the operations carry, instead of the edge and face labels, which they do not). A collapse
    /// or a swap that breaks it is refused. Defined in TopoOffsetTetMesh.cpp.
    bool repulsion_embedding_kept(const std::vector<size_t>& tids) const;
    /// Whether the simplex on the vertices `c` (2 or 3 of them) is in the input complex, from the
    /// cells' labels: in a cell of the complex, or, for offset_in, on a domain-boundary face of a
    /// body cell. Single-body offset_in or offset_out only, which repulsion_smoothing() checks.
    bool simplex_in_input_complex(const std::vector<size_t>& c) const;
    std::atomic<size_t> m_repulsion_embed_refused{0};
    /// Diagnostic (2026-09-28, Uday): where the front solves stop. Per pass, histograms of the
    /// final gradient norm of each front solve (log10 bins, -14..+7) and of its ratio to the
    /// solve's first gradient norm (log10 bins, -14..+1), read from polysolve's Criteria after
    /// the solve. Asked because 96% of front solves hit the 10-iteration cap under the
    /// tolerance-unit objective while the interior solves stop on the same absolute tolerance.
    static constexpr int kGradBins = 22;
    std::array<std::atomic<size_t>, kGradBins> m_front_grad_abs{}, m_front_grad_rel{};
    optimization::NewtonCounters m_newton_plastic;
    void log_smoothing_pass_accounting() override;

    /// DEBUG_crossings (log-only). The ring measure of every front vertex as the last pass left
    /// it (NaN where a vertex has none), the positions it was taken at, and whether it is current
    /// (consolidation renumbers vertices, so the loop clears it). A pass's line counts the front
    /// vertices whose ring measure went from <= 1 to > 1 (up), back (down), new vertices already
    /// over (a split's, or an old id at a new position after a storage retry), and vertices over
    /// the bar that the pass removed.
    std::vector<double> m_cross_ring;
    std::vector<Vector3d> m_cross_pos;
    bool m_cross_valid = false;
    /// Per smoothing pass: accepted front moves that took the moved vertex itself (own), or one of
    /// its front neighbours (neighbour), from a ring measure <= 1 to > 1.
    std::atomic<size_t> m_cross_own{0}, m_cross_neighbour{0};
    /// The exit test's ring measure of every vertex (energy_criterion()), face for face.
    std::vector<double> front_ring_measures() const;
    /// One vertex's ring measure, from its own live offset faces (NaN if it has none).
    double ring_measure_at(size_t vid) const;
    /// Take the snapshot; with `compare`, log the crossings against the previous one first.
    /// `match_positions`: an id counts as the same vertex only at the same position (operation
    /// passes, which move no vertex; a storage retry inside one renumbers).
    void crossing_snapshot(const std::string& pass, bool compare, bool match_positions);
    /// The engine calls this at the start of local_operations() and after each operation pass
    /// that ran; DEBUG_crossings hooks the operation passes here. Log-only.
    void update_attributes() override;

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
    /// The collapse energy rule's before-half: the max of tet_energy() over the one-rings of v1
    /// and v2, cached by collapse_edge_before() and compared by collapse_after_connectivity().
    mutable wmtk::threading::enumerable_thread_specific<double> m_collapse_energy_before;
    /// offset_collapse_changed_cells: the collapse in flight's before-energies, for the rule over
    /// the changed cells only (collapse_after_connectivity()). ring1_max is the max of
    /// tet_energy() over v1's ring, every cell of which the collapse reshapes or removes;
    /// v2_only holds (tid, tet_energy) of the cells of v2's ring outside v1's ring, sorted by tid:
    /// the collapse leaves their slots and vertices alone, so one of them changes energy only
    /// through a front face relabelled across it, and an unchanged one is in neither max.
    struct CollapseCells
    {
        double ring1_max = 0.;
        std::vector<std::pair<size_t, double>> v2_only;
    };
    mutable wmtk::threading::enumerable_thread_specific<CollapseCells> m_collapse_cells;
    /**
     * @brief The link of the collapsed edge, captured in collapse_before_vertex().
     *
     * Which vertices a collapse can move off the offset surface, exactly: the faces that DIE are
     * the ones carrying both endpoints, (v1, v2, w) for w in the link, so only v2 and those w can
     * lose their last surface face. A face (v1, a, b) with neither corner on the edge does not
     * die -- it is relabelled onto v2 -- so a and b keep it and are unaffected. Nothing but v2
     * can gain, since faces only ever move from v1 to v2.
     *
     * Captured before the collapse because the edge is gone by collapse_after_vertex(), which is
     * where the refresh runs.
     */
    mutable wmtk::threading::enumerable_thread_specific<std::vector<size_t>> m_collapse_edge_link;
    void log_smooth_trace() const;

    /// Which held faces are outside m_envelope, and by how much. Diagnostic only.
    void audit_surface_containment(const std::string& when) const;

    ////// wmtk::TetOptimizerMesh hooks

    /// Is this vertex on a region boundary -- a tag boundary, or the domain wall. Derived, not
    /// stored, exactly as in 2D.
    bool vertex_is_on_region(const size_t vid) const
    {
        return m_vertex_extra[vid].m_is_on_region || !m_vertex_attribute[vid].on_bbox_faces.empty();
    }

    /**
     * @brief The containment of a tracked face an operation has just made (the base asks only
     * after the fact). A wall face: m_envelope, band or not -- nothing leaves the domain box.
     * The offset surface: nothing in the loop, m_offset_envelope in the final pass. Any other
     * tracked face: m_envelope when it is held (face_is_held()), else nothing.
     */
    std::shared_ptr<SampleEnvelope> surface_envelope_for_face(
        const std::array<size_t, 3>& vids) const override
    {
        const auto found = try_tuple_from_face(vids);
        if (!found) return m_envelope; // not reached: every caller asks about an existing face
        const Tuple& f = std::get<0>(*found);
        if (f.switch_tetrahedron(*this) && face_is_offset_surface_live(f)) {
            return m_freeze_front ? m_offset_envelope : nullptr;
        }
        return face_is_held(f) ? m_envelope : nullptr;
    }

    /// Surface edges may be flipped, as a topology-preserving diagonal flip. Both tracked
    /// surfaces need it: the offset surface is re-triangulated constantly.
    bool allow_surface_swap() const override { return true; }

    /**
     * @brief The energy rule's EARLY half for a swap, on the engine's lower bound.
     *
     * `after` is the largest AMIPS^3 over the new cells, which the engine computes before this
     * component has labelled them (swap_after_cells()). tet_energy() of a cell is its AMIPS^3
     * plus a non-negative face term, so `after` not strictly below the before-maximum
     * (swap_before_interior() / swap_before_surface()) already decides what swap_after_cells()
     * would decide on the real cells: refused. Same rule, same counter, applied before the
     * rollback would cost anything (see collapse_quality_allowed() for the measurement). The
     * engine's `before` (stored AMIPS^3 of the old cells) is not read. The 4-4 / 5-6 case search
     * and the face swap's gate score the same energy earlier still, through
     * swap_edge_44_energy(), swap_edge_56_energy() and swap_face_before(). Off with
     * offset_swap_veto. No placement test: a swap moves no vertex.
     */
    bool swap_quality_allowed(double after, double /*before*/, bool) const override
    {
        if (!m_offset_params.offset_swap_veto) return true;
        if (weighted_amips(after) < m_swap_energy_before.local().max) return true;
        ++iter_cnt_swap_energy_reject;
        return false;
    }
    /**
     * @brief The 4-4 / 5-6 case search's score, on the per-tet energy: the max over `tets` of
     * tet_energy() for op_case 0 (the current cells, looked up by their vertex ids) and of
     * candidate_energy() for a candidate (op_case >= 1); std::numeric_limits<double>::max() as
     * soon as a cell is inverted, as the base does.
     *
     * TetMesh::swap_edge_44() and ::swap_edge_56() pick their retetrahedralization by seeding
     * `min_energy` with the score of DOING NOTHING (op_case 0) and taking a case only when it
     * scores strictly lower, before any after-hook runs. With this score that test IS the swap
     * rule of swap_after_cells() for these swaps, applied to cells that do not exist yet (see
     * tet_energy(), CANDIDATE CELLS); the after-hook applies it again on the real cells. Until
     * 2026-09-28 the search scored AMIPS^3 (the base's max over the cells), so a case that lowers
     * the energy while raising AMIPS never reached the rule.
     *
     * Also counts the scored cases of a flip of the offset surface for [flip funnel]
     * (SwapSurfaceSides::case0_energy: inverted, not better, better), and records the lowest
     * candidate score for the sanity check (SwapRecord::scored_energy).
     *
     * Until 2026-09-25 these overrides reported stop_energy for op_case 0, which turned the
     * base's test into "the new cells are under stop_energy" for flips of the offset surface
     * (the absolute bar; see swap_before_surface() for why it existed and why it went).
     */
    double swap_edge_44_energy(const std::vector<std::array<size_t, 4>>& tets, const int op_case)
        override;
    double swap_edge_56_energy(const std::vector<std::array<size_t, 4>>& tets, const int op_case)
        override;
    /**
     * @brief The face (2-3) swap's before-hook: TetOptimizerMesh::swap_face_before() with its
     * AMIPS^3 gate replaced by the per-tet energy.
     *
     * The base gates the three new cells, each by AMIPS^3 against the max of the two cells' stored
     * AMIPS^3, and only then calls swap_before_interior(). Here each new cell's candidate_energy()
     * must be strictly below max(tet_energy(t0), tet_energy(t1)), and swap_before_interior() runs
     * BEFORE the gate, since it fills the record candidate_energy() reads. The rest of the base's
     * steps are duplicated in order, reject kinds included (the engine has no energy hook for this
     * gate; one is a planned follow-up, after which this override goes).
     */
    bool swap_face_before(const Tuple& t) override;
    bool check_surface_topology() const override { return m_offset_params.perform_sanity_checks; }

    /// A held vertex is pulled to and contained in m_envelope -- a real SampleEnvelope, as the
    /// pull requires. Any other vertex, the front's included, is held by nothing.
    std::shared_ptr<SampleEnvelope> smoothing_energy_envelope(const size_t vid) const override
    {
        return vertex_is_held(vid) ? m_envelope : nullptr;
    }
    std::shared_ptr<SampleEnvelope> smoothing_containment_envelope(const size_t vid) const override
    {
        return vertex_is_held(vid) ? m_envelope : nullptr;
    }

    /**
     * @brief Placement of a front vertex: the shared smoother with the offset's options. See
     * FrontSmooth3d.cpp.
     */
    bool smooth_front_vertex(const Tuple& t);
    /**
     * @brief A repulsion vertex in the passes and rounds before the march (repulsion_smoothing()):
     * the shared smoother at the front's options, against the sum of the per-tet energy over its
     * ring -- w AMIPS^3, and O(v) once for every ring cell that carries it (repulsion_energy(),
     * repulsion_cell_term()); under offset_front_smooth_veto, the max of the per-tet energy over
     * its ring may not rise. See FrontSmooth3d.cpp.
     */
    bool smooth_repulsion_vertex(const Tuple& t);
    /**
     * @brief Every vertex off the front (and off the plastic medium): the shared smoother with
     * TetWild's options (TetOptimizerMesh::smooth_after()), except that it minimises the per-tet
     * energy -- smoothing_extra_energy(), w AMIPS^3 over the ring, the smoother adding no AMIPS
     * term of its own -- and the veto compares the max of tet_energy() over the ring, under the
     * same key (offset_smooth_veto) and on the same vertices as the engine's AMIPS^3 veto it
     * replaces. See Optimize3d.cpp.
     */
    bool smooth_nonfront_vertex(const Tuple& t);
    /// w AMIPS^3 over vid's one-ring, the per-tet energy's AMIPS part in the smoother's form
    /// (CubedAMIPSEnergy3D), w = offset_amips_weight.
    std::shared_ptr<polysolve::nonlinear::Problem> amips3_energy(size_t vid) const;
    /// ||grad F|| at front vertex vid along its move direction, F the objective
    /// smooth_front_vertex() minimises. +inf if unmeasurable.
    double front_vertex_normal_gradient(size_t vid) const;
    /// The line a front vertex is placed along: the field normal, or that normal projected into
    /// the boundary surface (onto its crease) where an input envelope holds it.
    Vector3d front_vertex_move_direction(size_t vid) const;
    /// |cos| between front_vertex_move_direction() and the field normal: 1 means the convergence
    /// test's 1-D step is the step toward the level set, 0 means it measures a direction that
    /// cannot reduce the distance. Debug-frame diagnostic; see write_vtu().
    double front_move_alignment(size_t vid) const;
    /// The vertex measure over the one bar: |relative_residual(x)| / front_conv_frac(), the
    /// face term's order-0 stencil at one corner. Infinite when unmeasurable. As in 2D.
    double front_vertex_conv_ratio(size_t vid) const;
    /**
     * @brief THE definition of "placed" for a vertex on the offset surface, applied to its
     * already-measured front_vertex_conv_ratio(): finite and within the one bar. Every caller that
     * asks "is the placement of this front vertex done" -- energy_criterion()'s vertex count and
     * the placed split of the faces over the bar, and the adaptive-smoothing stop -- goes through
     * here. Unmeasurable (a non-finite ratio) is NOT placed.
     */
    bool front_placed_by_ratio(const double ratio) const
    {
        return std::isfinite(ratio) && ratio <= 1.;
    }
    /**
     * @brief THE ONE FACE FUNCTION: the mean over the face's stencil of r^2, r =
     * relative_residual(q) / front_conv_frac() -- the squared face measure, in units of the
     * tolerance, 1 at the bar.
     *
     * Every reader of the face measure goes through it: the logs print its root,
     * energy_criterion()'s exit, refinement and ring measures, the debug frames' front_err_ratio
     * and front_ring_ratio, and the per-tet energy tet_energy(), which adds it to the band cell of
     * every live front face. -1 when unmeasurable (a sample whose relative_residual() is not
     * finite, which the euclidean field never has), +inf when the bar is not positive.
     *
     * The (a, b, c) form reads the field of the face's edge (a, b) (potential_for_edge()), as the
     * face measure always has; the energy passes its band cell's field and its corners sorted, so
     * a face's term does not depend on which cell or which operation asks for it.
     */
    double face_offset_term(size_t a, size_t b, size_t c) const;
    double face_offset_term(
        const OffsetPotential3D& pot,
        const Vector3d& pa,
        const Vector3d& pb,
        const Vector3d& pc) const;
    /// The field's unit direction at front vertex vid (zero where grad Phi vanishes).
    Vector3d front_vertex_normal(size_t vid) const;
    /// The objective of front vertex vid, as the smoother assembles it for a front vertex it
    /// places against the offset term (smoothing_extra_energy()): w AMIPS^3 over the one-ring
    /// (amips3_energy()), the rest-shape AMIPS of its plastic cells (rest_energy_for_vertex(),
    /// null without the plastic medium), and front_energy(). What the measure above
    /// differentiates.
    std::shared_ptr<polysolve::nonlinear::Problem> front_objective(size_t vid) const;
    /// Whether the smoother places vid against the offset term: a front vertex, outside the
    /// frozen-front final pass, that no envelope also holds (vertex_is_held()).
    bool vertex_carries_offset_term(const size_t vid) const
    {
        return !m_freeze_front && m_offset_potential && m_vertex_extra[vid].m_is_on_offset &&
               !vertex_is_held(vid);
    }
    /// The AMIPS weight the shared smoother uses at vid, amips_w in smooth_vertex_3d(): 1 for a
    /// vertex it places against the offset term, whose objective is the per-tet energy's two
    /// parts at 1:1 (AMIPS, and the offset term in units of the tolerance); w_amips otherwise,
    /// the engine's own balance against the input-surface envelope term.
    double smoother_amips_weight(const size_t vid) const
    {
        if (vertex_carries_offset_term(vid)) return m_offset_params.offset_amips_weight;
        return m_params.w_amips > 0 ? m_s_amips * m_params.w_amips : 1.0;
    }
    /// THE smoothing objective at vid, for every vertex the shared smoother places: the per-tet
    /// energy tet_energy() = w AMIPS^3 + O summed over vid's ring, up to the terms that do not
    /// move with vid. w AMIPS^3 over the ring (amips3_energy()) for every vertex -- the smoother
    /// adds no AMIPS term of its own, every caller passing opts.w_amips 0 -- plus the offset terms
    /// of the front faces at vid when it is a front vertex the loop places (front_energy(); null
    /// in the final pass and for a front vertex an input envelope also pins), or, in the passes
    /// before the march, a repulsion vertex's term once per ring cell that carries it
    /// (repulsion_energy(), repulsion_cell_term()). The ring's other
    /// front faces do not have vid as a corner, so their terms are constants here. Under
    /// deform_others the rest-shape AMIPS of the plastic cells in the ring is added, as before.
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
            sum->add_energy(front_energy(vid, potential_ptr_for(vid)));
        }
        if (const auto rest = rest_energy_for_vertex(vid)) sum->add_energy(rest);
        return sum;
    }

    // ------- deform_others: the medium outside the band deforms -------

    /// Plastic medium: under deform_others every background cell -- ambient and the other objects
    /// alike -- is plastic, its rest shape re-stamped before every operation group, so smoothing
    /// resists only the increment since the group started and the medium flows instead of behaving
    /// as an elastic solid glued to the walls. The band (label 2) and the complex (label 1) are
    /// not plastic; element quality in the medium is the operation passes' job.
    bool m_plastic_active = false; ///< deform_others, outside the final pass
    bool cell_is_plastic(size_t tid) const
    {
        // Everything outside the band: ambient, the other objects and the input complex's
        // interior alike -- one material. The complex's boundary is what its tube holds.
        return m_plastic_active && m_tet_attribute[tid].label != 2;
    }
    /// Stamp rest := current for every plastic cell; called before every operation group.
    void stamp_plastic_rests();
    /// The plastic vertex's smoothing: rest-shape AMIPS over its ring, nothing else.
    bool smooth_plastic_vertex(const Tuple& t);
    /// Stamp rest := the cell's current corner positions (oriented order). No-op for
    /// non-plastic cells.
    void stamp_rest_cell(size_t tid);
    /// The rest-shape AMIPS over the plastic cells of vid's one-ring, weighted like the
    /// shared smoother weights its AMIPS term at vid (smoother_amips_weight()); null when the ring
    /// has none.
    std::shared_ptr<polysolve::nonlinear::Problem> rest_energy_for_vertex(size_t vid) const;
    /// The offset term for a front vertex, at offset_term_weight(): StencilEnergy3D over its
    /// incident live front faces, whose value is sum_f O(f), the terms the per-tet energy carries.
    /// Defined in FrontSmooth3d.cpp.
    std::shared_ptr<polysolve::nonlinear::Problem> front_energy(
        size_t vid,
        const std::shared_ptr<const OffsetPotential3D>& pot) const;


    /// Samples per offset face; see offset_face_samples().
    int stencil_order() const { return m_offset_params.stencil_order; }
    /// How many points for_each_face_sample() visits at the configured order: 3 at order 0, and
    /// (n+1)(n+2)/2 + n^2 with n = 2^(k-1) above it, i.e. 4, 10, 31, 109, ... Kept in step with
    /// for_each_face_sample() by the unit test `stencil-order-point-counts`.
    int stencil_points_per_face() const
    {
        const int k = m_offset_params.stencil_order;
        if (k < 0) return 0;
        if (k == 0) return 3;
        const int n = 1 << (k - 1);
        return (n + 1) * (n + 2) / 2 + n * n;
    }

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

    /// A quantity sampled at the stencil points of an offset-surface face, corners included
    /// (for_each_face_sample()).
    struct FaceSamples
    {
        double max = 0.;
        double sum = 0.;
        size_t n = 0;
    };

    /**
     * @brief The stencil_order stencil a triangle is sampled on, handed to `visit` one point at
     * a time as (point, wa, wb, wc) with the barycentric weights that built it. Order 0 is the
     * three corners alone; order k >= 1 is the vertices of the triangle subdivided k-1 times by
     * 4-way midpoint refinement plus the centroid of each of its 4^(k-1) sub-triangles, so the
     * counts are 3, 4, 10, 31, 109 for k = 0..4 (stencil_points_per_face()). The corners are in
     * it on purpose: the quantity sampled is a distance to the level set, which at a corner is
     * that vertex's own placement error (see face_offset_term()).
     *
     * The weights are handed out for the front smoother: with the moving vertex x as p0, a
     * sample is wa*x + wb*p1 + wc*p2, so it moves by wa per unit of x; StencilEnergy3D keeps the
     * weights fixed while the points slide with x (stencil_face_at() in FrontSmooth3d.cpp).
     *
     * Takes positions rather than a Tuple: face_offset_term() reads a face's corners in sorted
     * order (see tet_energy()). The 2D twin is for_each_offset_edge_sample().
     */
    template <typename Visit>
    void for_each_face_sample(
        const Vector3d& p0,
        const Vector3d& p1,
        const Vector3d& p2,
        Visit&& visit) const
    {
        const int k = m_offset_params.stencil_order;
        if (k < 0) return;

        const auto emit = [&](const double wa, const double wb, const double wc) {
            visit(Vector3d(wa * p0 + wb * p1 + wc * p2), wa, wb, wc);
        };
        // Order 0 is the three CORNERS alone. That is the whole point of including them: the
        // measure sampled here is a distance to the level set, which at a corner is exactly that
        // vertex's own placement error, so one stencil covers what used to be two criteria.
        if (k == 0) {
            emit(1., 0., 0.);
            emit(0., 1., 0.);
            emit(0., 0., 1.);
            return;
        }
        // Order k >= 1: the vertices of the triangle subdivided k-1 times by 4-way midpoint
        // refinement, plus the centroid of each of its 4^(k-1) sub-triangles. With n = 2^(k-1)
        // segments per side that is (n+1)(n+2)/2 + n^2 points: 4, 10, 31, 109, ...
        const int n = 1 << (k - 1);
        const double dn = double(n);
        for (int i = n; i >= 0; --i) {
            for (int j = n - i; j >= 0; --j) {
                emit(double(i) / dn, double(j) / dn, double(n - i - j) / dn);
            }
        }
        // The sub-triangles, in integer barycentric coordinates over 3n. "Up" triangles have
        // corners (i+1,j,l), (i,j+1,l), (i,j,l+1) for i+j+l = n-1, so centroid (3i+1, 3j+1,
        // 3l+1); "down" triangles (i+1,j+1,l), (i,j+1,l+1), (i+1,j,l+1) for i+j+l = n-2, so
        // centroid (3i+2, 3j+2, 3l+2). n(n+1)/2 + n(n-1)/2 = n^2 of them.
        const double d3n = 3. * dn;
        for (int i = n - 1; i >= 0; --i) {
            for (int j = n - 1 - i; j >= 0; --j) {
                const int l = n - 1 - i - j;
                emit((3. * i + 1.) / d3n, (3. * j + 1.) / d3n, (3. * l + 1.) / d3n);
            }
        }
        for (int i = n - 2; i >= 0; --i) {
            for (int j = n - 2 - i; j >= 0; --j) {
                const int l = n - 2 - i - j;
                emit((3. * i + 2.) / d3n, (3. * j + 2.) / d3n, (3. * l + 2.) / d3n);
            }
        }
    }

    /// The same lattice over a face the mesh carries. Visitor signature as above.
    template <typename Visit>
    void for_each_offset_face_sample(const Tuple& f, Visit&& visit) const
    {
        const auto vs = get_face_vids(f);
        for_each_face_sample(
            m_vertex_attribute[vs[0]].m_posf,
            m_vertex_attribute[vs[1]].m_posf,
            m_vertex_attribute[vs[2]].m_posf,
            std::forward<Visit>(visit));
    }

    /// The Phi residual at the `stencil_order` stencil's points of offset face `f`.
    /// Returns nothing for a face with an unreachable corner. The 2D twin is
    /// offset_edge_samples().
    FaceSamples offset_face_samples(const Tuple& f) const;

    /**
     * @brief The convergence criterion's own split: ||grad (Phi - c)^2|| at band vertices plus
     * the face-interior chord diagnostic. Same fields as the 2D GradientSplit, face samples in
     * place of edge samples.
     */
    struct GradientSplit
    {
        double max_reachable = 0., avg_reachable = 0.;
        double max_pinned = 0.;
        size_t n_reachable = 0, n_pinned = 0;
        double max_at_vertex = 0., max_in_face = 0.;
        size_t n_face_samples = 0;
        size_t n_skipped_inverted = 0, n_skipped_unrounded = 0;
    };
    /// @param include_face_samples false skips the face-interior half (the expensive one).
    GradientSplit gradient_split(bool include_face_samples = true) const;

    /**
     * @brief The "energy_gradient" criterion: the front's Newton-step ratios and the refinable
     * faces. The 3D twin of TopoOffsetTriMesh::EnergyCriterion.
     */
    struct EnergyCriterion
    {
        double max_vertex = 0., max_face = 0.; ///< ratios to the bar (1 = bar)
        /// Running sums of the SAME ratios, over the same measurable simplices the maxima are
        /// taken over, so avg_vertex() / avg_face() below are the plain means of what max_vertex
        /// / max_face report the largest of. Reported only; nothing tests them.
        double sum_vertex = 0., sum_face = 0.;
        double bar = 1.;
        size_t n_vertices = 0, n_faces = 0, n_unmeasurable = 0;
        size_t worst_vid = static_cast<size_t>(-1);
        /// Reported only: the faces over the bar, split by whether all three corners are PLACED
        /// (front_placed_by_ratio(), the one notion). A face whose corners are placed and whose
        /// centroid still misses the level set is under-resolved -- the state the vertex test
        /// cannot see, and the only one the sag rule may act on: refining a face whose corners
        /// are still moving would chase the front rather than resolve it.
        size_t n_faces_over = 0, n_faces_over_placed = 0;
        double max_face_placed = 0.;
        Vector3d worst_placed_centroid = Vector3d::Zero();
        double tube = 0.;
        /// Faces over the bar that the refinement does not take: no chord target below the
        /// largest sizing scalar at their corners. Like every face over the bar they block the
        /// exit. Two states share this count (see energy_criterion()): corners at the sizing
        /// floor, which nothing can refine, and a longest edge still at least twice the target
        /// length at the corners, which the split pass shortens. The first is n_corners_at_floor.
        size_t n_at_floor = 0;
        double max_face_at_floor = 0.; ///< the worst of them, as a ratio to the bar
        Vector3d worst_at_floor_centroid = Vector3d::Zero();
        double worst_at_floor_scalar = 0.; ///< the largest sizing scalar at the worst one's corners
        /// Of n_at_floor, the faces whose largest corner scalar IS the floor: they cannot be
        /// refined, so a run keeping them cannot converge. The loop warns with them every turn
        /// they exist, and the verdict and the throw_on_nonconvergence message quote the same
        /// sentence, sizing_floor_fact().
        size_t n_corners_at_floor = 0;
        double max_face_corners_at_floor = 0.; ///< the worst of them, as a ratio to the bar
        Vector3d worst_corners_at_floor_centroid = Vector3d::Zero();
        /// The floor, max(min_sizing_scalar, min_edge_length / l), and which of the two it is.
        double floor_scalar = 0.;
        bool floor_from_min_edge_length = false;
        size_t n_unplaced = 0; ///< measurable front vertices that front_placed_by_ratio() refuses
        /// A face over the bar with all three corners placed: a, b are the ends of its LONGEST
        /// edge (the chord the target is derived from), c the third corner; len the longest
        /// edge's length.
        ///
        /// `measure` is the face's MEAN sag as a LENGTH (the ratio times the tube). Nothing
        /// reads it today -- refinement is the halving, which needs the face's corners alone --
        /// and it is kept because the sag condition is still being reworked.
        struct Refinable
        {
            size_t a, b, c;
            double measure, len;
        };
        std::vector<Refinable> refinable;
        /// THE RING MEASURE, filled only under front_measure "vertex_ring". At a front vertex v,
        /// over the offset faces incident to v that the face loop measured (three front corners):
        ///
        ///     r_v = sqrt( (1/n_v) sum_f O(f) ),   O(f) = face_offset_term()
        ///
        /// over the n_v such faces, every face weighted equally. This is the front smoother's own
        /// offset term at v, normalised by the ring size so that the bar keeps its meaning:
        /// StencilEnergy3D at offset_term_weight() is sum_f O(f) = n_v r_v^2, the same faces'
        /// terms the per-tet energy (tet_energy()) carries. Exact where the face's field
        /// (potential_for_edge()) is the vertex's (potential_for()), always so with one region,
        /// and the field is euclidean (StencilEnergy3D charges (Phi - c)/c, which is
        /// relative_residual() only there); the smoother's objective also carries the AMIPS term
        /// beside this one.
        /// Built from the very face_offset_term() calls the face loop makes, so both modes judge
        /// identical face numbers. A vertex with any unmeasurable incident face has no ring
        /// measure (n_rings_unmeasurable; the face itself is already in n_unmeasurable). Until
        /// 2026-09-28 faces were weighted by area here and in the smoother; the per-tet energy has
        /// no area in it, so neither has this. Why it exists:
        /// the face exit and refinement are per face, the smoother minimises per vertex ring, and
        /// the two disagree at the margin -- on the deliverable cube at target_distance_rel 1e-3 /
        /// front_conv_rel 1e-5 the face exit never fired, turns 12-15 each ending with a handful
        /// of faces at 1.00x to 1.19x the bar, the smoothing having nudged faces from 0.99x to
        /// just over the bar while lowering the ring they belong to.
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
        Vector3d worst_ring_at_floor_pos = Vector3d::Zero();
        bool rings_ok() const { return max_ring <= bar; }
        double avg_ring() const { return n_rings ? sum_ring / double(n_rings) : 0.; }
        /// Every front vertex placed: the VERTEX measure, a DIAGNOSTIC only. Counted through
        /// front_placed_by_ratio() rather than re-derived from max_vertex, so the reported count
        /// and the per-vertex notion cannot drift apart. Nothing in the exit or the verdict tests
        /// it since 2026-09-25; see converged().
        bool vertices_ok() const { return n_unplaced == 0; }
        bool faces_ok() const { return max_face <= bar; }
        /// THE exit test, and the front half of the run's verdict: every offset face's measure
        /// within the bar AND nothing unmeasurable. One quantity, the FACE measure
        /// (the root of face_offset_term()), now decides smoothing (it is the front smoothing energy,
        /// StencilEnergy3D), refinement (which faces enter `refinable`) and termination -- the
        /// decision of 2026-09-25. The vertex measure left the exit then and is reported only.
        /// The stencil contains the corners, so a face within the bar bounds its corners' error
        /// in the RMS sense over the stencil, not each corner separately.
        ///
        /// No refinable.empty() term: a refinable face is over the bar, so faces_ok() already
        /// implies that nothing is refinable. Against the old exit (every vertex placed, nothing
        /// unmeasurable, nothing refinable) two things change: a face over the bar that the
        /// refinement does not take (n_at_floor) used to let the run end "converged" and now
        /// blocks the exit, the loop warning when its corners are at the sizing floor; and a
        /// front vertex over the bar no longer blocks it once every face it is a corner of is
        /// within the bar.
        ///
        /// Under front_measure "vertex_ring" the ring measure takes the face measure's place here:
        /// every front vertex's ring measure within the bar AND nothing unmeasurable, the face
        /// measure then reported only. An unmeasurable ring has an unmeasurable face in it, which
        /// n_unmeasurable counts, so the second half is the same test in both modes. In one
        /// statement for both: max over front vertices v of the VERTEX measure
        /// V(v) within the bar, nothing unmeasurable -- under "face" V(v) is the max of the face
        /// measure over v's offset faces with three front corners, so the max over vertices is
        /// the max over those faces, faces_ok(); under "vertex_ring" V(v) is the ring measure,
        /// rings_ok().
        bool converged() const
        {
            return (ring_exit ? rings_ok() : faces_ok()) && n_unmeasurable == 0;
        }
        /// The n_at_floor faces as one sentence, for the turn's line (a warning when some have
        /// their corners at the sizing floor), the verdict and the throw_on_nonconvergence
        /// message alike, so all three state the same fact. Empty when n_at_floor is 0. Under
        /// front_measure "vertex_ring" the same three places get the n_rings_at_floor vertices
        /// instead, empty when there are none.
        std::string sizing_floor_fact() const;
        double ratio() const { return bar > 0. ? std::max(max_vertex, max_face) / bar : 0.; }
        /// Means over the measurable front vertices / offset faces; 0 when there are none.
        double avg_vertex() const { return n_vertices ? sum_vertex / double(n_vertices) : 0.; }
        double avg_face() const { return n_faces ? sum_face / double(n_faces) : 0.; }
    };
    EnergyCriterion energy_criterion();
    /// The edge length that would bring a front chord's sag under the tube: 3/4 L
    /// (tube / sag)^(1/p) capped at L/2, with the exponent p measured from how the level set
    /// turns across the chord. Same formula as 2D.
    ///
    /// Reached from ONE place now: the refinable / at-floor test in energy_criterion(), which
    /// asks whether there is any target left below what the face's corners already carry.
    double front_chord_target(size_t va, size_t vb, double len, double sag, double tube) const;

    /// THE refinement: halve the sizing scalar at the corners of every refinable face, once per
    /// vertex per call, floored at max(min_sizing_scalar, min_edge_length / l), then graded
    /// outward. Returns the number of vertices lowered.
    size_t refine_front_by_halving(const std::vector<EnergyCriterion::Refinable>& faces);
    /// The same halving at the listed vertices themselves: each lowered once per call, floored,
    /// then graded. The face form above is this on its faces' corners, in the order given;
    /// front_measure "vertex_ring" calls it directly with the vertices whose ring measure is over
    /// the bar.
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
        /// Max front_vertex_conv_ratio over the measurable front vertices (1 = the bar, the
        /// turn criterion's own test).
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
    SmoothingProgress smoothing_progress(const std::vector<Vector3d>& before);
    /// The interleaved smoothing of one operation group under adaptive_smoothing: one pass at a
    /// time through local_operations({0,0,0,1}), each followed by smoothing_progress(), until
    /// the front has converged (max ratio <= 1) or stalled (max ratio fell by less than
    /// adaptive_smoothing_stall_rel) AND the background has settled (max step <=
    /// adaptive_smoothing_step_rel x its target edge), or adaptive_smoothing_max_passes.
    void smooth_group_to_convergence(const char* group_name);
    /// The energy criterion as measured when the loop converged; the final pass runs after
    /// it and the verdict must not be re-measured on that mesh.
    std::optional<EnergyCriterion> m_energy_verdict;

    /// Turn a residual_split()'s outside-support tally into the hard error.
    void report_outside_support(const char* when, const DistanceSplit& s) const;
    /// Whether vid is a band vertex the optimizer could still place at target_distance. An
    /// envelope-held offset vertex is pinned, as is one on the domain boundary. Same rule as 2D.
    bool band_vertex_is_reachable(const size_t vid) const
    {
        if (m_vertex_extra[vid].m_is_on_offset && vertex_is_held(vid)) return false;
        return !vertex_is_on_domain_boundary(vid);
    }

    /// TetWild's stall-driven sizing refinement, verbatim; final pass only. See the 2D twin.
    size_t refine_sizing_around_worst(double max_metric) override;

    /// Why the final pass is stuck: a census of the tets stuck-refine is about to chase. The 3D
    /// twin of log_stuck_refine_census().
    void log_stuck_refine_census(double max_metric, double filter_energy);

    /// For every element above `filter_energy`, why its edges cannot be split: short / valence /
    /// contain / free. The 3D twin of log_refine_block_census().
    void log_refine_block_census(const std::string& when, double filter_energy) const;

    /**
     * @brief The energy rule's EARLY half, on the engine's lower bound.
     *
     * The engine scores each reshaped cell before the collapse exists and hands its AMIPS^3 in
     * as `q`. tet_energy() of that cell is q plus a non-negative face term, so q alone above the
     * before-maximum (collapse_before_vertex()) already decides: the rule that
     * collapse_after_connectivity() applies on the real cells would refuse too. Refusing here
     * costs nothing; refusing there costs the collapse and its rollback -- measured 2026-09-28 on
     * the cube at 10 threads, 400000-650000 refused collapses per late turn, every pass 2-3x
     * slower than the AMIPS-only rule that refused before executing. Same rule, same counter,
     * applied as soon as it can be. The engine's own `ring_max` (AMIPS^3 over v1's ring) is not
     * read; TetWild's exemption for an unrounded v1 is kept. Off with offset_collapse_veto.
     */
    bool collapse_quality_allowed(size_t v1, double q, double /*ring_max*/) const override
    {
        if (!m_offset_params.offset_collapse_veto || !m_vertex_attribute.at(v1).m_is_rounded) {
            return true;
        }
        if (weighted_amips(q) <= m_collapse_energy_before.local()) return true;
        ++iter_cnt_collapse_energy_reject;
        return false;
    }

    mutable std::atomic<size_t> m_deg_split_created{0};
    size_t m_deg_prev_split_created = 0;

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

    /// Max of the two normalized criteria on this face: AMIPS over stop_energy for the cells it
    /// separates, and on a live offset face the root of face_offset_term() (the face's RMS
    /// relative error over its stencil in units of the bar; +inf if unmeasurable). > 1 means it
    /// fails at least one. The coarsen-mode collapse accept reads it.
    double face_criterion_rel(const Tuple& f) const;
    /// AMIPS of a cell over stop_energy -- the 3D twin of TriOptimizerMesh::quality_rel().
    double cell_quality_rel(const size_t tid) const;
    /// ... and the worst of the (up to two) cells a face separates.
    double amips_rel_at_face(const Tuple& f) const;

    /**
     * @brief Put the optimization's frames on the run's single debug timeline (see
     * write_debug_frame()), labelled "r<turn><tag><pass>_<op>" / "r<turn><tag>_end", tag S in
     * the loop and F in the final pass; led by m_frame_prefix (i during
     * init_optimize).
     */
    void write_optimization_debug_output(const std::string& path) override
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
        write_debug_frame(label);
    }
    /// One line of <output>_frames.txt; truncates the file on the first frame.
    void append_frame_label(size_t idx, const std::string& label) const;
    /**
     * @brief One frame of the run's single debug timeline: <output>_NNNNN.vtu with the next
     * sequence number, and one "NNNNN<tab>label" line in <output>_frames.txt. Every debug
     * frame the run writes -- the input as loaded, the construction stages, and the
     * optimization's own frames through write_optimization_debug_output() -- goes through
     * this sequence, so the numbers are consecutive and the .txt says what each
     * one is. The only debug files outside it are the ones that are not this mesh:
     * <output>_input_complex.vtu and the phi grid.
     */
    void write_debug_frame(const std::string& label);

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

    /// Construction, start to finish, on the input mesh as given: the simplicial embedding,
    /// marching_tets(), the re-embedding and the offset tagging. The optimization is
    /// optimize_offset(), which the driver calls afterwards.
    void construct_offset(const std::filesystem::path& output_file);

    /// Marching tets: every edge with one endpoint in the input complex (label 1/2) and the
    /// other in the background (label 0) is split -- where d(x) = target_distance along the edge,
    /// or under construction_mode's fallback at half the maximum marchable distance or at the
    /// midpoint (see edge_split_sphere_trace()) -- and afterwards every background tet
    /// still touching a complex frontier vertex (the split-off halves) becomes the band
    /// (label 2).
    void marching_tets();

    /// repulsion_smoothing_passes and repulsion_rounds (see the spec): between the simplicial
    /// embedding and marching_tets(), smoothing passes and then rounds of the loop's operations
    /// that push the outer ends of the marched edges out to 2 x target_distance, stopping once
    /// every outer end is beyond target_distance + front_conv.
    void repulsion_smoothing();
    /// Every construction label (vertex, edge, face, tet) back to 0, then label_input_complex():
    /// the input complex from the region tags, which the operations carry.
    void relabel_input_complex();

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

    /// INTERIOR swaps only: the ring must be homogeneous in tag and in construction label, and
    /// the single value of each is captured for swap_after_cells() to stamp on the new cells. A
    /// ring that is not homogeneous has a region boundary or the offset surface running through
    /// it, and an interior swap would move that boundary.
    ///
    /// DO NOT call this on the surface path. A face is on the offset surface exactly when one
    /// incident cell is band and the other is not (cell_is_offset_band: label == 2), so the ring
    /// around a surface-flip edge ALWAYS spans two labels and this always refuses -- which is
    /// what made every offset-surface flip impossible until 2026-09-17. The surface path uses
    /// swap_capture_surface_sides() instead.
    bool swap_capture_tag(const std::vector<size_t>& tids);
    /// The tag swap_after_cells writes onto the tets an INTERIOR swap created, chosen in `before`.
    wmtk::threading::enumerable_thread_specific<CellTag> m_swap_tag;
    /// The construction label shared by every cell of an interior swap's ring, captured alongside.
    wmtk::threading::enumerable_thread_specific<int> m_swap_label;

    /// SURFACE flips: the two old surface faces split the edge ring into two arcs, each
    /// homogeneous in tag and label, and the flip keeps both -- it only moves the diagonal
    /// between them. A ring vertex strictly inside an arc identifies that arc's side; the flip's
    /// four named vertices a, b, c, d do not, because a and b are the flipped edge and c and d
    /// sit on the interface between the arcs. So each remaining ring vertex is mapped to the
    /// (tag, label) of the cells it belongs to, and swap_after_cells() stamps each new cell from
    /// a ring vertex it contains. Mirrors SimWildMesh::swap_before_surface(), which solves the
    /// same problem for its tags; the offsets carry a construction label too, so both travel.
    ///
    /// Refuses only when one ring vertex is seen with two different sides, which is a genuinely
    /// inconsistent neighbourhood rather than the ordinary two-sided ring.
    struct SwapSurfaceSides
    {
        std::map<size_t, std::pair<CellTag, int>> by_vertex;
        /// a, b, c, d as prepare_surface_flip named them. The flip's net surface change is
        /// -(a,b,c) -(a,b,d) +(a,c,d) +(b,c,d), so these four are exactly the vertices whose
        /// membership it can change, and swap_after_cells() refreshes them.
        std::array<size_t, 4> abcd{};
        /// [flip funnel]: this flip is a flip of the offset surface. Read at each later stage to
        /// follow the flip through. Cleared at the top of swap_before_surface() and of
        /// swap_before_interior(), so none of these fields outlives its flip.
        bool worthwhile = false;
        /// [flip funnel]: the case search's score of the current cells (op_case 0), which every
        /// scored case of this flip has to beat. Written by swap_edge_44_energy() /
        /// swap_edge_56_energy().
        double case0_energy = 0.0;
        /// [flip funnel]: set the first time swap_edge_44_energy() / swap_edge_56_energy() is
        /// asked to score a CANDIDATE case (op_case >= 1) for this flip, so the funnel counts
        /// flips for which the base found at least one retetrahedralization that survives
        /// swap_edge_*_accept_case(), not cases. Never set for a 3-2, which has no case search.
        bool saw_case = false;
        /// tids.size() as swap_before_surface() saw it: 3, 4 or 5, i.e. which swap this is.
        int kind = 0;
    };
    bool swap_capture_surface_sides(
        const std::vector<size_t>& tids,
        size_t a,
        size_t b,
        size_t c,
        size_t d);
    wmtk::threading::enumerable_thread_specific<SwapSurfaceSides> m_swap_sides;
    /**
     * @brief The swap energy rule's before-half, per thread; swap_before_interior() and
     * swap_before_surface() fill it, swap_after_cells() compares against it.
     *
     * `max` is the max of tet_energy() over the cells the swap replaces and over `outside`.
     * `outside` holds the band cells outside the ring across a face of the ring's boundary that
     * the swap hands to a cell of the other side, since their energy changes although the swap
     * does not make them: only a 3-2 surface flip has such faces, (a,c,d) and (b,c,d), which are
     * faces of its old cell (a,b,c,d) and afterwards of new cells on the other side. Every other
     * boundary face keeps a cell of its own side, and the cells beyond it keep their energy.
     */
    struct SwapEnergyBefore
    {
        double max = 0.;
        std::vector<size_t> outside;
    };
    mutable wmtk::threading::enumerable_thread_specific<SwapEnergyBefore>
        m_swap_energy_before; // read by the const early half
    /**
     * @brief The swap in flight on this thread, as candidate_energy() reads it (see tet_energy(),
     * CANDIDATE CELLS). Filled by swap_record_fill(), which swap_before_interior() and
     * swap_before_surface() call once they have accepted the ring -- in the engine's
     * swap_edge_before(), swap_edge_44_before() and swap_edge_56_before(), all of which run before
     * the connectivity changes and so before the 4-4 / 5-6 case search, and in swap_face_before()
     * before its gate. Cleared at the top of both hooks, so no record outlives its swap into the
     * next one on the thread.
     *
     * A swap keeps every face on the boundary of the union U of the cells it replaces, each held
     * afterwards by exactly one new cell with the same cell outside it, and moves no vertex. So
     * the live front faces of the new cells are
     *   - a boundary face of U whose new cell is band and whose outside cell is neither band nor
     *     input complex (or absent). An interior swap's new cells all take the ring's one label
     *     (swap_capture_tag()); a flip's new cell takes the side of a ring vertex it contains
     *     (swap_after_cells()), which for a boundary face holding a ring vertex is that vertex's
     *     side, and for one on a, b, c, d alone -- the 3-2's (a,c,d), (b,c,d), whose old cell
     *     was on the OTHER side -- the side of the new cell's apex, so the entry names the
     *     band-side apexes;
     *   - for a 4-4 or 5-6 flip, the created faces (a,c,d), (b,c,d), between the two sides inside
     *     U, when one side is band and the other neither band nor input complex. Two new cells
     *     hold each, and the entry names the band-side apexes.
     * A 3-2 flip whose old cell (a,b,c,d) was the band side hands (a,c,d), (b,c,d) to the cells
     * outside U, which it does not create (SwapEnergyBefore::outside); no candidate is scored for
     * a 3-2, which has no case search. Each term is face_offset_term() on the field of the band
     * cell that carries the face now, or, for a face whose band side is new, of the first band
     * cell of U. The unit test swap-candidate-record checks every prediction against the swapped
     * mesh built separately: 4-4 and 3-2 flips at four side labelings, an interior 4-4, a face
     * swap.
     */
    struct SwapRecord
    {
        bool active = false;
        std::vector<size_t> verts; ///< the vertices of U, sorted
        struct Face
        {
            std::array<size_t, 3> face; ///< sorted
            std::vector<size_t> apexes; ///< empty: any new cell holding the face is its band side
            double term = 0.; ///< face_offset_term(); negative: unmeasurable
        };
        std::vector<Face> faces;
        /// perform_sanity_checks: the score the case search or the face gate gave the
        /// configuration the engine commits -- for a 4-4 / 5-6 the lowest candidate score, since
        /// the engine takes the strictly lowest and does not say which; for a face swap the max of
        /// its three cells. swap_after_cells() compares it with the max tet_energy() of the cells
        /// it gets. Not set for a 3-2, which scores nothing before it exists.
        bool scored = false;
        double scored_energy = std::numeric_limits<double>::max();
        void clear()
        {
            active = false;
            verts.clear();
            faces.clear();
            scored = false;
            scored_energy = std::numeric_limits<double>::max();
        }
    };
    mutable wmtk::threading::enumerable_thread_specific<SwapRecord> m_swap_record;
    /// The record over the cells `tids` a swap replaces; `flip` for a surface flip, whose sides
    /// swap_capture_surface_sides() has captured first. See SwapRecord.
    void swap_record_fill(const std::vector<size_t>& tids, bool flip);
    /// perform_sanity_checks, from swap_after_cells() once the new cells are labelled: the
    /// record's scored_energy against max_tet_energy(tids), counted in m_swap_scoring_*.
    void swap_scoring_check(const std::vector<size_t>& tids, double after);

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
};


} // namespace wmtk::components::topological_offset
