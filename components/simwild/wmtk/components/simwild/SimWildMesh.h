#pragma once

#include <optional>

#include <wmtk/MeshData.hpp>
#include <wmtk/utils/FeatureInputs.hpp>

#include <igl/Timer.h>
#include <wmtk/SurfaceTagAttributes.h>
#include <wmtk/TetMesh.h>
#include <wmtk/TetOptimizerMesh.h>
#include <wmtk/utils/PartitionMesh.h>
#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/envelope/CompositeEnvelopes.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <wmtk/threading/parallel_for.hpp>

#include "ConnectedComponent.hpp"
#include "Parameters.h"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <igl/remove_unreferenced.h>
#include <atomic>
#include <functional>
#include <memory>
#include <unordered_set>

#include <wmtk/utils/SurfaceTopology.hpp>
#include <wmtk/utils/partition_utils.hpp>
#include "expression_parser/Expression.hpp"

namespace wmtk::components::simwild {

/// The shared attribute types live on the base; keep the unqualified names working. The
/// simwild copies were identical apart from tetwild's m_is_on_open_boundary, which simwild
/// never sets.
using VertexAttributes = wmtk::TetOptimizerMesh::VertexAttributes;
using FaceAttributes = wmtk::TetOptimizerMesh::FaceAttributes;

class TetAttributes
{
public:
    /**
     * cubed (!) AMIPS quality
     */
    double m_quality;
    /**
     * All image labels. Stored as pairs of image ID and the tag within the image. Using a sparse
     * vector, so 0 entries are ommitted.
     */
    CellTag tags;
};

class SimWildMesh : public wmtk::TetOptimizerMesh
{
public:
    using ExprPtr = expression_parser::ExpressionPtr;

    size_t m_tags_count = 0;
    std::map<int64_t, std::string> m_tag_id_to_name;
    std::map<std::string, int64_t> m_tag_name_to_id;
    /// Names of the face tags: bit i of a face's input set is m_face_tag_names[i].
    std::vector<std::string> m_face_tag_names;
    // The names of the feature curves' inputs are the base's m_feature_edge_names, set by the
    // driver: the label of the i-th curve input, or the edge tags of a tagged mesh input
    // (InputData::feature_edge_names).

    /**
     * @brief (Re)build the base's m_input_envelopes: one envelope per label, and each surface
     * face held by those of the labels it lies on:
     * bit i < F of a face's input set is face tag i (m_face_tag_names, F of them), bit F + j the
     * boundary of tet tag j's region. A border between tet tag sets S and S' carries the bits
     * of the tags in one and not the other; ambient is the empty set, so a border of region X with
     * ambient is held by X's envelope, one between X and Y by both. As tetwild holds a tracked
     * face to its inputs' envelopes, and topological_offset's 2D mesh a region boundary to its
     * tags'.
     *
     * Member b is built around the same surface as m_envelope, restricted to label b -- the
     * input triangles of that label in generation, else the mesh's faces carrying it -- at the
     * eps and with the predicate m_envelope has at the time (init_label_envelopes()), and follows
     * it: at eps_simplify while the simplification runs, around the simplified surface after.
     * Built around the faces embedded on the inputs instead, the members cost +33 to +70% time;
     * around the input triangles, +2 to +6%. m_envelope stays the union, for the engine's direct
     * uses of it. A surface face carrying no label throws, as in tetwild (the base's
     * surface_envelope_for_face()).
     *
     * With one union envelope a collapse by the seam of two sheets side by side moved a face of
     * one 0.125 off it (eps 0.029), within eps of the other. Two volumes a fraction of eps apart
     * did not drift so with it (each region's boundary stayed within 0.7 eps of its own box,
     * the tets between them in the way); their labels' envelopes make that hold by construction.
     */
    void init_label_envelopes(double eps, bool exact);

    /// The face-tag bits of an input set: those below the number of face tags.
    InputSet face_tag_mask() const
    {
        const size_t n = m_face_tag_names.size();
        return n >= size_t(SurfaceTagAttributes::max_inputs) ? ~InputSet(0)
                                                             : (InputSet(1) << n) - 1;
    }

    /// Per row of m_F_envelope: its label bits, when m_F_envelope is the inputs' triangles
    /// (InputData::F_envelope_labels) or the mesh's own surface faces at load
    /// (init_surfaces_and_boundaries(): each face's input set); empty once simplify() rebuilds
    /// m_F_envelope around the simplified faces. init_label_envelopes() reads an empty one as
    /// "around the mesh's faces as they are then": at load that left the per-label envelopes to
    /// follow the faces, so after the simplification under envelope_reference "input" they were
    /// rebuilt around the simplified faces while the union was rebuilt around the input.
    std::vector<InputSet> m_F_envelope_labels;
    /// Per row of m_F_envelope, when it was built from the mesh's own surface faces
    /// (init_surfaces_and_boundaries(), simplify()): that face's input set; empty otherwise. What
    /// envelope_data() labels the rows by where m_F_envelope_labels is empty (after simplify()).
    std::vector<InputSet> m_F_envelope_face_sets;

    // A reference an `envelope` input gave (InputData::envelope_reference), the first
    // component's labelled input, is the base's (TetOptimizerMesh::m_has_reference, which states
    // the guarantee): set by the driver before init, with m_V_envelope, m_F_envelope and
    // m_F_envelope_labels its triangles of this mesh's labels, m_reference_curves its feature
    // curves, and m_input_curves set from both. init_surfaces_and_boundaries() then throws for a
    // label a held face or feature edge carries that the reference lacks, and, for a remeshing,
    // which holds the mesh to it, for a held face or feature edge received outside its labels'
    // envelopes (check_reference_containment()): nothing is rebuilt around the mesh, as the
    // envelope check without a reference does.

    /// The label of bit `bit` of a face's input set, as the output names it: a face tag's name, a
    /// tet tag's label (io::collect_tags()).
    std::string label_name(size_t bit) const override;
    /**
     * @brief The envelope the run hands on (io::append_envelope_block()), when it built its own:
     * m_F_envelope's rows labelled by m_F_envelope_labels, else by m_F_envelope_face_sets, with
     * the feature curves of `curves` (edge_input: the bit of `curve_names`) and its points.
     * Unlabelled, and without curves and points, where the rows' labels are not known: a bare
     * `envelope` given, or the EnvelopeSurface group of a .msh.
     */
    MeshData envelope_data(
        const utils::FeatureInputs& curves,
        const std::vector<std::string>& curve_names) const;
    /**
     * @brief After a tag operation, the envelope it hands on, around its result: m_F_envelope
     * every face the next component holds, labelled (m_F_envelope_labels) as it labels it -- its
     * face tags, and the tet tags on one side of it and not the other, the regions it bounds
     * (init_surfaces_and_boundaries()) -- on its own vertices in m_V_envelope; none without such
     * a face. A tag operation retags cells and splits edges, so the faces between regions after
     * it are new geometry, which no envelope given covers: the driver does not read one, and the
     * next component holds these faces to themselves.
     */
    void set_envelope_around_tags();

    /// The base holds only wmtk::OptimizerParameters; this is the same object, typed, for the
    /// simwild-only fields (tags, quality/sizing fields, the operation name).
    Parameters& m_sim_params;
    std::vector<Vector3d> m_V_envelope;
    std::vector<Vector3i> m_F_envelope;
    double m_envelope_eps = -1;

    std::vector<std::tuple<ExprPtr, double>> m_sizing_field;
    std::vector<std::tuple<ExprPtr, double>> m_quality_field;

    bool m_collapse_check_quality = true;


    // When set, split_edge_after binary-searches vmid onto the zero-crossing of this function.
    // Negative = stays on v1 side, positive = stays on v2 side.
    std::function<double(const Vector3d&)> m_voronoi_split_fn = nullptr;

    SimWildMesh(Parameters& _m_params, double envelope_eps, int _num_threads = 0)
        : wmtk::TetOptimizerMesh(_m_params, nullptr)
        , m_sim_params(_m_params)
    {
        m_envelope_eps = envelope_eps;
        NUM_THREADS = _num_threads;
        p_tet_attrs = &m_tet_attribute;
        m_collapse_check_link_condition = false;
        m_collapse_check_manifold = false;

        // solver is lazily created on first use

        optimization::deactivate_opt_logger();

        m_s_envelope = 1. / (m_params.eps * m_params.eps);

        double& wa = m_params.w_amips;
        double& we = m_params.w_envelope;
        we = 1 - wa;
        logger().info("w_envelope = {}", we);
    }

    ~SimWildMesh() {}
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    TetAttCol m_tet_attribute;

    double cell_quality(const size_t tid) const override { return m_tet_attribute[tid].m_quality; }
    void set_cell_quality(const size_t tid, const double q) override
    {
        m_tet_attribute[tid].m_quality = q;
    }
    bool allow_surface_swap() const override { return m_sim_params.allow_surface_swap; }
    bool check_surface_topology() const override { return m_sim_params.check_surface_topology; }

    // only used with unit tests
    void create_mesh_attributes(
        const std::vector<VertexAttributes>& _vertex_attribute,
        const std::vector<TetAttributes>& _tet_attribute)
    {
        const size_t n_tet = _tet_attribute.size();
        m_vertex_attribute.resize(_vertex_attribute.size());
        m_face_attribute.resize(4 * n_tet);
        m_tet_attribute.resize(n_tet);

        for (size_t i = 0; i < _vertex_attribute.size(); i++)
            m_vertex_attribute[i] = _vertex_attribute[i];

        // Keep whatever init() reserved. AttributeCollection::resize only ever grows, so the
        // calls above cannot shrink a collection -- but assigning m_attributes directly does,
        // and it used to drop the tet attributes to exactly n_tet. The attributes have to stay
        // at least as large as the connectivity: an operation that creates a new element
        // indexes them by its id, and a 5->6 swap creates one. tetwild hit the same thing (see
        // the note on its copy); there the overrun wrote a double past the end and libc++ let
        // it pass, here it assigns a std::set and segfaults outright.
        const size_t tcap = std::max(n_tet, m_tet_attribute.size());
        m_tet_attribute.m_attributes = std::vector<TetAttributes>(tcap);
        for (size_t i = 0; i < n_tet; i++) m_tet_attribute[i] = _tet_attribute[i];
        for (size_t i = 0; i < n_tet; i++)
            m_tet_attribute[i].m_quality = get_quality(tuple_from_tet(i));
    }

    // TODO This should not be here but inside wmtk
    // TODO This should not be here but inside wmtk
    void init_envelope(const MatrixXd& V, const MatrixXi& F);

    CellTag string_set_to_cell_tag(const std::set<std::string>& str_set);

    void set_sizing_field(const nlohmann::json& sizing_field_json);

    void set_quality_field(const nlohmann::json& quality_field_json);

    double target_quality(const size_t tid) const;
    double target_quality(const Tuple& t) const;
    /// SimWild's target is per tag, so this normalizes each cell by its own before the shared
    /// optimizer compares one cell against another. See TetOptimizerMesh::quality_rel.
    double quality_rel(const size_t tid) const override;
    double quality_rel(const Tuple& t) const;
    bool check_mesh_quality(double& max_rel_quality, const bool verbose = false) const;
    /**
     * @brief Verify that every interface between unlike tags carries a surface face.
     *
     * Checks that direction only: an interface face that is not tagged is a violation, while a
     * tagged face between like-tagged tets is not reported here.
     *
     * @param verbose Log a summary line, and the first few offending faces.
     * @return true if every interface face is tagged.
     */
    bool check_interface_faces_tagged(const bool verbose = false) const;

    /**
     * @brief Update the attributes of the mesh after an iteration of operations.
     *
     * Re-derives the surface from the tet tags over the whole mesh: a face is a surface face
     * exactly when it separates two differently tagged tets, and a vertex is on the surface
     * exactly when it belongs to such a face. This is the global form of what the operations
     * used to attempt incrementally; doing it here, between passes and single-threaded, avoids
     * both the stale flags that OR-merging leaves behind and the neighbourhood reads that are
     * unsafe inside a parallel operation.
     *
     * @see check_interface_faces_tagged, which verifies the post-condition in both directions.
     */
    void update_attributes() override;
    /// Out of the tet whose tags come first (std::set order) into the other: a sheet between two
    /// tag sets is oriented the same way all over, from the tags alone.
    using TetOptimizerMesh::orient_tracked_face;
    std::array<size_t, 3> orient_tracked_face(
        const std::array<size_t, 3>& f,
        const std::vector<size_t>& tets) const override;
    /// A tet's tags: the regions of a volume history are the tag sets.
    std::vector<int64_t> tet_tags(size_t tid) const override;

    std::vector<size_t> active_vertices() const override;


    ////// Attributes related


public:
    void simplify();

    /**
     * @brief Check if all vertices of the mesh are rounded.
     *
     */
    bool all_rounded() const;

    //
    //
    // debug use
    std::atomic<int> cnt_split = 0, cnt_collapse = 0;

protected:
    std::tuple<double, double> optimization_quality_stats() override;
    double optimization_stop_metric() const override { return 1.; }
    bool optimization_stop_at_float() const override { return m_sim_params.stop_at_float; }
    void write_optimization_debug_output(const std::string& path) override { write_vtu(path); }

    bool collapse_quality_allowed(size_t v1, double quality, double ring_max) const override;

    bool split_before_cells(const Tuple& edge, const std::vector<Tuple>& parents) override;
    bool split_after_cells(size_t v1, size_t v2, size_t v_new, const std::vector<Tuple>& children)
        override;
    bool split_adjust_position(size_t v_new, const std::vector<Tuple>& children) override;
    /// A face split puts the new vertex at m_face_split_position (the face's centroid if unset),
    /// in doubles, so every corner must be rounded. The children of each tet on the face take its
    /// attributes; the face's three pieces take the face's, and the tets' other faces keep
    /// theirs, and every old edge keeps its feature tags. Refused if a child is inverted.
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
    /// A tet split puts the new vertex at the tet's centroid, in doubles, so every corner must be
    /// rounded. The four children take the tet's attributes and its four faces and six edges keep
    /// theirs; the six faces inside it are new. Refused if a child is inverted. Only
    /// tight_seal_geom's simplicial embedding of its regions splits tets.
    bool split_tet_before(const Tuple& t) override;
    bool split_tet_after(const Tuple& t) override;

    bool swap_before_interior(const std::vector<size_t>& tids) override;
    bool swap_before_surface(
        const std::vector<size_t>& tids,
        size_t a,
        size_t b,
        size_t c,
        size_t d) override;
    bool swap_after_cells(const std::vector<size_t>& tids, bool is_surface_flip) override;

private:
    // tight_seal_geom's helpers that need the mesh, defined in TightSealGeom.cpp
    /// BVH of the boundary of the tets of `tids` for which in_set holds: their interfaces with
    /// the other tets and their faces on the mesh boundary. From a point outside the set, the
    /// distance to the set is the distance to this boundary. Returns false if the set is empty.
    bool tight_seal_boundary_bvh(
        const std::vector<size_t>& tids,
        const std::function<bool(size_t)>& in_set,
        SimpleBVH::BVH& bvh) const;
    /// Split edge e, keeping `cells` up to date: the children of a tet in it replace it, and
    /// each child takes its parent's flag in `filled` (resized to the new capacity), if given its
    /// entry in `cell_group` (resized with -1), and if given its parent's `corner_labels` for the
    /// corners it keeps (resized). Returns the new vertex, or -1 if the split was refused.
    size_t tight_seal_split_tracked(
        const simplex::Edge& e,
        std::unordered_set<size_t>& cells,
        std::vector<char>& filled,
        std::vector<int>* cell_group = nullptr,
        std::vector<CornerLabels>* corner_labels = nullptr);
    /// Split face f at p, keeping `cells`, `filled`, `cell_group` and `corner_labels` up to date
    /// as tight_seal_split_tracked does. Returns the new vertex, or -1 if the split was refused.
    size_t tight_seal_split_face_tracked(
        const std::array<size_t, 3>& f,
        const Vector3d& p,
        std::unordered_set<size_t>& cells,
        std::vector<char>& filled,
        std::vector<int>* cell_group = nullptr,
        std::vector<CornerLabels>* corner_labels = nullptr);
    /// The dilation surface S of tight_seal_geom: the level set d(x, R) = radius, R + B's boundary,
    /// as a BVH of triangles. On each edge of a tet outside R (region < 0) between a vertex with
    /// dist_R_at < radius and one with dist_R_at >= radius, the crossing is sphere traced on dist_R
    /// from the near end (to within tol, on R's side); the crossings of each tet are joined into
    /// its cut polygon (a triangle or a quad, in two triangles). Returns false if S is empty; n_traced: edges traced, n_failed: traces that left
    /// their edge (the far end is taken).
    bool tight_seal_dilation_surface_bvh(
        const std::vector<size_t>& tids,
        const std::vector<int>& region,
        const std::vector<double>& dist_R_at,
        const std::function<double(const Vector3d&)>& dist_R,
        double radius,
        double tol,
        SimpleBVH::BVH& bvh,
        size_t& n_traced,
        size_t& n_failed) const;
    /// How many of `tids` have a vertex on the mesh's boundary, a face with no tet beyond it.
    size_t tight_seal_cells_on_boundary(const std::vector<size_t>& tids) const;
    /// Split tet tid at its centroid, keeping `cells`, `filled`, `cell_group` and `corner_labels`
    /// up to date as tight_seal_split_tracked does. Returns the new vertex, or -1 if the split was
    /// refused.
    size_t tight_seal_split_tet_tracked(
        size_t tid,
        std::unordered_set<size_t>& cells,
        std::vector<char>& filled,
        std::vector<int>* cell_group = nullptr,
        std::vector<CornerLabels>* corner_labels = nullptr);
    /// tight_seal_geom's "split_fill" mode. Each tet of `components` labels its corners: a corner
    /// takes the nearest of the component's regions (the first on an exact tie), except that the
    /// corners of a face the tet shares with a region's tet take that region (a tet has at most
    /// one such face, the regions' union being simplicially embedded). From these labels, all
    /// taken before any split: the tets with three or four labels are split at their centroid,
    /// then the faces with three in a tet holding them at theirs, then the edges whose ends differ
    /// in a tet holding them where the distances to the two labels are equal (not if that falls on
    /// an end). Each child keeps its parent's labels of the corners it keeps, a new vertex has
    /// none, and each tet of the components then takes a region by tight_seal::split_fill_label.
    /// tags_before[id] gets the tags of each tet it labels (filled[id] set) as they were before.
    /// No topology is checked.
    void tight_seal_split_fill(
        const std::vector<ConnectedComponent>& components,
        const std::vector<std::vector<size_t>>& components_regions,
        const std::vector<int64_t>& tags,
        const std::vector<SimpleBVH::BVH>& region_bvhs,
        std::vector<char>& filled,
        std::vector<CellTag>& tags_before);
    /// tight_seal_geom's topology check: for each tag the fill added to or took from a tet, warns
    /// if (change in non-manifold vertices, islands, merges) is not (0, 0, 0). The tag before the
    /// fill and after it are compared on the output mesh, so splits that only refine a tag change
    /// nothing. Non-manifold vertices by wmtk::utils::nonmanifold_vertices, both ends of a
    /// non-manifold edge included; an island is a component of the tag after the fill (through
    /// faces) with none of its tets from before; a merge joins two components from before into
    /// one. Also warns if a component from before is split or lost (the fill took the tag from a
    /// tet carrying it with another tag). filled and tags_before as split_fill leaves them.
    void tight_seal_topology_check(
        const std::vector<char>& filled,
        const std::vector<CellTag>& tags_before) const;
    /// tight_seal_geom's `sanity` checks, on the output: throws if a tet is inverted, a filled
    /// tet (filled[id] set) carries other than exactly one of `tags`, or a part of the filled
    /// tets, connected through faces, borders fewer than two of them.
    void tight_seal_geom_sanity(const std::vector<int64_t>& tags, const std::vector<char>& filled)
        const;

    ////// Operations

    // Where split_face() puts the new vertex; the face's centroid if unset. Set before
    // split_face(), cleared after.
    std::optional<Vector3d> m_face_split_position;
    size_t m_last_face_split_vertex = 0;
    // split_face_before() to split_face_after(). Face splits are serial (only tight_seal_geom
    // makes them).
    struct FaceSplitCache
    {
        std::array<size_t, 3> v; // the face
        FaceAttributes face;
        std::vector<std::pair<size_t, TetAttributes>> tets; // by each tet's corner off the face
        std::vector<std::pair<std::array<size_t, 3>, FaceAttributes>> faces; // the tets' others
        std::map<std::array<size_t, 2>, FeatureEdgeAttributes> changed_edges;
        Vector3d position;
        double sizing_scalar = 1;
        size_t partition_id = 0;
        std::vector<int> on_bbox_faces;
    };
    FaceSplitCache m_face_split_cache;
    // split_tet_before() to split_tet_after(). Tet splits are serial (only tight_seal_geom makes
    // them).
    struct TetSplitCache
    {
        std::array<size_t, 4> v; // the tet
        TetAttributes tet;
        std::vector<std::pair<std::array<size_t, 3>, FaceAttributes>> faces; // its four faces
        std::map<std::array<size_t, 2>, FeatureEdgeAttributes> changed_edges;
        Vector3d position;
        double sizing_scalar = 1;
        size_t partition_id = 0;
        std::vector<int> on_bbox_faces;
    };
    TetSplitCache m_tet_split_cache;

    struct SplitTagCache
    {
        size_t v_new = 0;
        /// Parent cell data, keyed by the edge opposite the split edge.
        std::map<simplex::Edge, TetAttributes> tets;
    };
    wmtk::threading::enumerable_thread_specific<SplitTagCache> split_tag_cache;

    struct SwapTagCache
    {
        CellTag tet_tags;
        std::map<size_t, CellTag> ring_tags;
    };
    wmtk::threading::enumerable_thread_specific<SwapTagCache> swap_tag_cache;

public:
    /**
     * @brief Init from meshes image.
     *
     * @param V #Vx3 vertices of the tet mesh
     * @param T #Tx4 vertex IDs for all tets
     * @param T_tags #Tx1 image data represented by the individual tets
     */
    void init_from_image(
        const MatrixXr& V,
        const MatrixXi& T,
        const MatrixSi& T_tags,
        const std::vector<std::string>& tag_names,
        const MatrixXi& F_tagged = MatrixXi(),
        const MatrixSi& F_tags = MatrixSi(),
        const std::vector<std::string>& face_tag_names = {},
        const utils::FeatureInputs* features = nullptr,
        const utils::EmbedFeaturesResult* features_placed = nullptr,
        const MatrixSi& F_orientation = MatrixSi());
    void init_from_image(
        const MatrixXd& V,
        const MatrixXi& T,
        const MatrixSi& T_tags,
        const std::vector<std::string>& tag_names,
        const MatrixXi& F_tagged = MatrixXi(),
        const MatrixSi& F_tags = MatrixSi(),
        const std::vector<std::string>& face_tag_names = {},
        const utils::FeatureInputs* features = nullptr,
        const utils::EmbedFeaturesResult* features_placed = nullptr,
        const MatrixSi& F_orientation = MatrixSi());

    /// `features`, optional: the input feature curves and points, and where the insertion put
    /// them (`features_placed`, in the vertex numbering of the mesh), tagged and anchored here.
    void init_surfaces_and_boundaries(
        const utils::FeatureInputs* features = nullptr,
        const utils::EmbedFeaturesResult* features_placed = nullptr);
    /// Tracks the faces F_tagged (rows of T's vertices, as read_mesh_data gives them) as surface,
    /// with input set bit j where F_tags has a 1 in column j, oriented along the row, or with
    /// F_orientation as it says (InputData::F_orientation). Throws on a row that is no face.
    void init_face_tags(
        const MatrixXi& F_tagged,
        const MatrixSi& F_tags,
        const std::vector<std::string>& face_tag_names,
        const MatrixSi& F_orientation = MatrixSi());


    /**
     * @brief Escape a stuck max energy by refining the sizing field around the
     * worst elements.
     *
     * Finds the m_params.stuck_refine_num_worst tets with the highest energy,
     * gathers all vertices within m_params.stuck_refine_rings graph rings of
     * them, and multiplies each such vertex's m_sizing_scalar by
     * m_params.stuck_refine_factor (clamped at m_params.stuck_refine_min_scalar).
     * Then runs gradation_smooth_sizing so the refined region blends smoothly
     * into the surrounding resolution. Replaces the old global
     * adjust_sizing_field mechanism. Returns the number of vertices refined.
     */
    size_t refine_sizing_around_worst(double max_metric = 0.) override;


    /**
     * @brief Checks if an edge COULD be an open boundary edge.
     *
     * The method performs two checks. First, it checks if the two vertices are marked as on the
     * open boundary. Second, it checks if the edge is within the open boundary envelope.
     * Note that these checks are not sufficient to guarantee that an edge is actually on the open
     * boundary! For example, an almost degenerate triangle with two edges on the open boundary
     * could cause a false positive result.
     */
    bool is_order_2_edge(const Tuple& e) const;
    bool is_order_2_edge(const std::array<size_t, 2>& e) const;

    /// `cell_fields`: extra cell fields, each a name and a value per tet id (missing ids write 0).
    void write_vtu(
        const std::string& path,
        MeshMap* output = nullptr,
        const std::vector<std::pair<std::string, std::vector<double>>>& cell_fields = {});

    void write_surface(const std::string& path) const;

public:
    // substructure functions

    /**
     * @brief Compute the vertex order for every vertex.
     */
    void init_vertex_order();

public:
    // Annotations

    double tet_volume(const size_t tid) const;

    /**
     * @brief Find all connected components that contain the `tag_in` tags.
     */
    std::vector<ConnectedComponent> compute_connected_components(const CellTag& tag_in) const;
    std::vector<ConnectedComponent> compute_connected_components(const ExprPtr& expr) const;

    /**
     * @brief Find all regions that do not contain the tags from `tag_in`.
     *
     * The `tag_in` vector represents a list of tag intersections.
     * Example: tag_in = {{1,2},{3}}
     * A face will be considered as a hole if its tags neither include {1,2} or {3}.
     * The following would be holes:
     * {}
     * {1,4}
     * The following would be NOT holes:
     * {1,2,4} <- contains 1 and 2
     * {3} <- contains 3
     * {1,3} <- contains 3
     *
     * The returned vector also contains "holes" that touch the boundary. They should be
     * ommitted in hole filling.
     */
    std::vector<ConnectedComponent> find_holes(const std::vector<CellTag>& tag_in) const;

    /**
     * @brief Compute the boundary of a tag.
     *
     * @param tag A set of tags that must be present in a tet for being considered as
     * tagged.
     * @param V Vertices of the tag boundary.
     * @param F Faces of the tag boundary.
     */
    void compute_tag_boundary(const CellTag& tag, MatrixXd& V, MatrixXi& F) const;

    /**
     * @brief Keep only the largest connected component for each of the distinct tag_0 values,
     * and engulf all other components.
     *
     * @param lcc_tags
     * @param n_lcc The number of largest components that should be kept.
     */
    void keep_largest_connected_component(
        const std::vector<CellTag>& lcc_tags,
        const size_t n_lcc = 1);

    void fill_holes_topo(
        const std::vector<CellTag>& fill_holes_tags,
        double threshold = std::numeric_limits<double>::infinity());

    void seal_connected_components(
        const std::vector<CellTag>& tag_sets,
        const std::vector<ConnectedComponent>& components);

    void tight_seal_topo(
        const std::vector<std::vector<CellTag>>& tight_seal_tag_sets,
        double threshold = std::numeric_limits<double>::infinity());

    /**
     * @brief Geometric tight seal between the regions R_i, the tets tagged with `tags[i]` and
     * nothing else. The 3D counterpart of SimWildMeshTri::tight_seal_geom.
     *
     * J is the closing of the union R of the R_i by the ball B of diameter `distance`, minus R:
     * the part of E that B cannot reach. First R is made simplicially embedded (no tet outside R
     * has all its vertices on R; wmtk/utils/SimplicialEmbedding.hpp): each tet outside R whose four
     * faces lie on R is split at its centroid, then each face outside R whose three edges do, then
     * each edge outside R joining two vertices of R at its midpoint. Without it, a gap one tet wide
     * would be in J at any distance, every vertex of R counting as inside. Then J is found in three
     * steps.
     *  1. The dilation surface S, d(x, R) = distance/2, sphere traced on the edges that cross it.
     *     The mesh's boundary is no part of S, so where R + B reaches it, it acts as a wall.
     *  2. A vertex is inside if it is in R + B and d(v, S) > distance/2, counted from
     *     (1 - trace_rel_tol) x distance/2 up, so a vertex on the level set is never split off
     *     by rounding; every vertex of R is inside (R lies in the closing; S, linear in each
     *     tet, can pass closer to R than the level set). Every tet outside R with a vertex
     *     inside is processed: each of its edges from an inside vertex to an outside one is
     *     split where it leaves J, at d(x, S) = distance/2, sphere traced from the inside end
     *     (to within trace_rel_tol x distance/2; not split if the inside end is already that
     *     close or closer), and J is the processed tets whose vertices are all inside.
     *  3. The connected components of J (through faces) are kept by the rule of `selection`.
     * The operation throws if a tet of J has a vertex on the mesh's boundary, in any component,
     * filled or not. Regions more than distance/2 from the boundary keep J off it.
     * Each component (connected through faces, so two touching at an edge or a vertex are filled
     * separately) is then split as its tets' own labels of their corners require, and each
     * resulting tet takes the region its corners' labels give (tight_seal_split_fill). No
     * topology is checked.
     *
     * `selection` "explicit": the regions are `tags`, two or more, which may not hold ambient
     * (the tag named "ambient"). "detected": `tags` is ignored (with a warning if it is not
     * empty), the regions are every tag but ambient that some tet has alone, and each part that
     * borders at least `min_regions` of them (2 or more; ignored, with a warning if not 2, for
     * "explicit") is filled with those it borders.
     *
     * `spatial_selection`: J keeps only the tets whose tags satisfy it, before its components are
     * found; nullptr keeps them all. On hold: the simwild component passes none, and refuses
     * any tight_seal_geom_spatial_selection but its default.
     *
     * `sanity`: afterwards, throw if a tet is inverted, a filled tet carries other than one
     * region's tag, or a part of the filled tets borders fewer than two regions (a concave
     * feature of one body, filled).
     */
    void tight_seal_geom(
        const std::vector<int64_t>& tags,
        double distance,
        double trace_rel_tol = 0.01,
        const std::string& selection = "explicit",
        int64_t min_regions = 2,
        bool sanity = false,
        const ExprPtr& spatial_selection = nullptr);

    void resolve_overlaps(const std::vector<std::array<ExprPtr, 2>>& intersecting_tags);

    void replace_tags(const std::vector<CellTag>& tags_in, const std::vector<CellTag>& tags_out);

    /**
     * @brief Gives tags priority over others.
     *
     * If a tet has multiple tags, only the one with the
     * highest priority will be kept. The priority is determined by the order of the tags in the
     * input vector, e.g., if tag A is before tag B in the vector, then A has higher priority than
     * B.
     *
     * @param tags A vector of tags, where the order determines the priority.
     */
    void tag_priority(const std::vector<int64_t>& tags);
};

} // namespace wmtk::components::simwild
