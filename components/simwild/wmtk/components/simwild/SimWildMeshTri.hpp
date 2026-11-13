#pragma once

#include <wmtk/MeshData.hpp>

#include <wmtk/TriOptimizerMesh.h>

#include <wmtk/SurfaceTagAttributes.h>
#include <limits>
#include <optional>
#include <unordered_set>

#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/optimization/solver.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/write_triangle_mesh.h>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include "ConnectedComponent.hpp"
#include "Parameters.h"
#include "expression_parser/Expression.hpp"


namespace wmtk::components::simwild::tri {

/// The attribute types live on the shared base; keep the unqualified names working. The
/// simwild copies were identical to triwild's apart from `m_feature_id`, which simwild leaves
/// at NO_FEATURE (it has no 0-dimensional features), and spelling `CellTag` for what is the
/// same `std::set<int64_t>`.
using VertexAttributes = wmtk::TriOptimizerMesh::VertexAttributes;
using EdgeAttributes = wmtk::TriOptimizerMesh::EdgeAttributes;
using FaceAttributes = wmtk::TriOptimizerMesh::FaceAttributes;

class SimWildMeshTri : public wmtk::TriOptimizerMesh
{
public:
    using ExprPtr = expression_parser::ExpressionPtr;

    /// The base holds only wmtk::OptimizerParameters; this is the same object, typed, for the
    /// simwild-only fields (tags, the operation name, the raw input box).
    Parameters& m_sim_params;

    std::vector<Vector2d> m_V_envelope;
    std::vector<Vector2i> m_E_envelope;
    /// Per row of m_E_envelope: its label bits, when m_E_envelope is the input curves
    /// (InputData::F_envelope_labels); empty once m_E_envelope is rebuilt from the mesh.
    std::vector<InputSet> m_E_envelope_labels;
    /// Per row of m_E_envelope, when it was built from the mesh's own surface edges
    /// (init_surfaces_and_boundaries()): that edge's input set; empty otherwise. What
    /// envelope_data() labels the rows by where m_E_envelope_labels is empty. The 2D twin of
    /// SimWildMesh::m_F_envelope_face_sets.
    std::vector<InputSet> m_E_envelope_edge_sets;
    // A reference an `envelope` input gave, held to as given, is the base's
    // (TriOptimizerMesh::m_has_reference): set by the driver before init, with m_V_envelope,
    // m_E_envelope and m_E_envelope_labels its segments of this mesh's labels, and
    // m_input_curve_points its curve ends and junctions.

    /// The label of bit `bit` of an edge's input set, as the output names it: an edge tag's name,
    /// a triangle tag's label (io::collect_tags()).
    std::string label_name(size_t bit) const override;
    /// The envelope the run hands on, when it built its own: the 2D twin of
    /// SimWildMesh::envelope_data(), with the input `points`.
    MeshData envelope_data(const std::vector<Vector2d>& points) const;
    /// After a tag operation, the envelope it hands on, around its result: the 2D twin of
    /// SimWildMesh::set_envelope_around_tags(), every edge the next component holds, labelled
    /// by its edge tags and the triangle tags on one side of it and not the other.
    void set_envelope_around_tags();
    /// The vertices of the input points (the point inputs), set before init: anchored there, in a
    /// ball of feature_envelope_ratio x eps (TriOptimizerMesh::anchor_feature_points()).
    std::vector<size_t> m_input_point_vids;

    /// Names of the edge tags: bit i of an edge's input set is m_edge_tag_names[i].
    std::vector<std::string> m_edge_tag_names;

    /// The edge-tag bits of an input set: those below the number of edge tags.
    InputSet edge_tag_mask() const
    {
        const size_t n = m_edge_tag_names.size();
        return n >= size_t(SurfaceTagAttributes::max_inputs) ? ~InputSet(0)
                                                             : (InputSet(1) << n) - 1;
    }

    /**
     * @brief The labels, one envelope each (the base's m_input_envelopes), and each surface
     * edge held by those of the labels it lies on: bit i < C of an edge's input set is edge tag
     * i (m_edge_tag_names, C of them), bit C + j the boundary of triangle tag j's region. An
     * edge tag carries its own bit. A border between triangle tag sets S and S' carries the bits
     * of the tags in one and not the other; ambient is the empty set, so a border of region X
     * with ambient is held by X's envelope, one between X and Y by both. The 2D twin of
     * SimWildMesh::init_label_envelopes().
     *
     * Member b is built around the same curves as m_envelope, restricted to label b -- the
     * input curves of that label in generation, else the mesh's surface edges carrying it
     * (TriOptimizerMesh::init_input_envelopes()) -- at `eps` and with the predicate `exact`, the
     * ones m_envelope has at the time, and follows it: at eps_simplify while the simplification
     * after the insertion runs (simplify()), around the simplified curves after it under
     * envelope_reference "simplified". m_envelope stays the union, for the engine's direct uses
     * of it. A surface edge carrying no label throws, as in triwild (the base's
     * surface_envelope_for_edge()). At most 64 edge tags and triangle tags together: a mesh with
     * more throws.
     */
    void init_label_envelopes(double eps, bool exact);

    std::vector<std::tuple<ExprPtr, double>> m_sizing_field;
    std::vector<std::tuple<ExprPtr, double>> m_quality_field;

    bool m_collapse_check_link_condition = false; // classical link condition
    bool m_collapse_check_topology = false; // sanity check
    bool m_collapse_check_manifold = false; // manifoldness check after collapse

    SimWildMeshTri(Parameters& _m_params, double envelope_eps, int _num_threads = 0)
        : wmtk::TriOptimizerMesh(_m_params)
        , m_sim_params(_m_params)
    {
        m_envelope_eps = envelope_eps;
        NUM_THREADS = _num_threads;

        optimization::deactivate_opt_logger();

        double& wa = m_params.w_amips;
        double& we = m_params.w_envelope;
        we = 1 - wa;
        logger().info("w_envelope = {}", we);
    }

    ~SimWildMeshTri() {}

public:
    /**
     * @brief Init from meshes image.
     *
     * @param V #Vx3 vertices of the tet mesh
     * @param T #Tx4 vertex IDs for all faces
     * @param T_tags #Tx1 image data represented by the individual faces
     * @param tag_names Names for each tag in T_tags. The size must be the same as the number of
     * columns in T_tags.
     */
    void init_from_image(
        const MatrixXd& V,
        const MatrixXi& T,
        const MatrixSi& T_tags,
        const std::vector<std::string>& tag_names,
        const MatrixXi& E_tagged = MatrixXi(),
        const MatrixSi& E_tags = MatrixSi(),
        const std::vector<std::string>& edge_tag_names = {},
        const MatrixSi& E_orientation = MatrixSi());

    /**
     * @brief Same, from EXACT input positions.
     *
     * Taken when the 2D arrangement produced a vertex with no double representation -- a
     * crossing between two input segments generally has none. Rounds what it can and leaves
     * the rest rational; the optimization loop reclaims them.
     */
    void init_from_image(
        const MatrixXr& V,
        const MatrixXi& T,
        const MatrixSi& T_tags,
        const std::vector<std::string>& tag_names,
        const MatrixXi& E_tagged = MatrixXi(),
        const MatrixSi& E_tags = MatrixSi(),
        const std::vector<std::string>& edge_tag_names = {},
        const MatrixSi& E_orientation = MatrixSi());

    void init_surfaces_and_boundaries();
    /// Tracks the edges E_tagged (rows of T's vertices, as read_curves gives them) as surface,
    /// with input set bit j where E_tags has a 1 in column j. Throws on a row that is no edge.
    /// The 2D twin of SimWildMesh::init_face_tags.
    void init_edge_tags(
        const MatrixXi& E_tagged,
        const MatrixSi& E_tags,
        const std::vector<std::string>& edge_tag_names,
        const MatrixSi& E_orientation = MatrixSi());

    void init_envelope(const MatrixXd& V, const MatrixXi& F);

    /**
     * @brief Simplify after the insertion: collapse every edge with the curve envelopes at
     * eps_simplify, under preserve_topology (the exact inversion check and the substructure link
     * condition), then hold the optimization to the envelopes envelope_reference names. The 2D
     * twin of SimWildMesh::simplify(); run_2D calls it where run_3D calls that one.
     */
    void simplify();

    CellTag string_set_to_cell_tag(const std::set<std::string>& str_set);

    void set_sizing_field(const nlohmann::json& sizing_field_json);

    void set_quality_field(const nlohmann::json& quality_field_json);

    double target_quality(const size_t tid) const;
    double target_quality(const Tuple& t) const;
    /// SimWild's target is per tag, so this normalizes each face by its own before the shared
    /// optimizer compares one face against another. See TriOptimizerMesh::quality_rel.
    double quality_rel(const size_t tid) const override;
    double quality_rel(const Tuple& t) const;
    bool check_mesh_quality(double& max_rel_quality, const bool verbose = false) const;

    /**
     * @brief Verify that every interface between unlike tags carries a surface edge.
     *
     * The 2D counterpart of SimWildMesh::check_interface_faces_tagged. A SimWild surface IS the
     * interface between unlike cell tags, so for any two triangles sharing an edge whose `tags`
     * differ, that edge must be marked `m_is_surface_fs` -- and, in the other direction, a
     * marked edge must separate unlike tags.
     *
     * Boundary edges (one incident triangle) have no second tag to differ from; a marked one is
     * counted and reported apart from the interior edges rather than as a violation.
     *
     * @param verbose Log a summary line, and the first few offending edges.
     * @return true if every interface edge is tagged and no tagged edge sits between like tags.
     */
    bool check_interface_edges_tagged(const bool verbose = false) const;

    /**
     * @brief Update the attributes of the mesh after an iteration of operations.
     *
     * The 2D counterpart of SimWildMesh::update_attributes: walks the edges already marked as
     * surface and clears the ones whose two triangles no longer disagree on their tags, then
     * rebuilds `m_is_on_surface` from the edges that survive. Like the 3D version it only
     * retires stale flags -- an edge that was never a surface edge does not become one here --
     * and it runs between passes, single-threaded, where reading both sides of an edge is safe.
     *
     * @see check_interface_edges_tagged, which verifies the post-condition in both directions.
     */
    void update_attributes() override;
    std::vector<size_t> active_vertices() const override;

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


    /// `cell_fields`: extra cell fields, each a name and a value per face id (missing ids write 0);
    /// `vertex_fields` likewise per vertex id.
    void write_vtu(
        const std::string& path,
        MeshMap* output = nullptr,
        const std::vector<std::pair<std::string, std::vector<double>>>& cell_fields = {},
        const std::vector<std::pair<std::string, std::vector<double>>>& vertex_fields = {}) const;
    void write_vtu_with_energies(const std::string& path) const;

public:
    /**
     * @brief A vector containing the vertex position and all positions of the surface neighbors.
     *
     * Returns an empty vector if vertex is not on the surface.
     */
    std::vector<Vector2d> get_surface_assembles(const Tuple& t) const;
    std::shared_ptr<polysolve::nonlinear::Problem> get_envelope_energy(const Tuple& t) const;

    std::vector<std::array<double, 6>> get_amips_assembles(const Tuple& t) const;
    std::shared_ptr<polysolve::nonlinear::Problem> get_amips_energy(const Tuple& t) const;

    /**
     * For debugging purposes.
     */
    void log_total_surface_energy();


    double triangle_area(const size_t fid) const;

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
     * The returned vector also contains "holes" that touch the boundary. They should be ommitted in
     * hole filling.
     */
    std::vector<ConnectedComponent> find_holes(const std::vector<CellTag>& tag_in) const;

    /**
     * @brief Compute the boundary of a tag.
     *
     * @param tag A set of tags that must be present in a triangle for being considered as tagged.
     * @param V Vertices of the tag boundary.
     * @param E Edges of the tag boundary.
     */
    void compute_tag_boundary(const CellTag& tag, MatrixXd& V, MatrixXi& E) const;

    /**
     * @brief Keep only the largest connected component for each of the distinct tag_0 values, and
     * engulf all other components.
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
     * @brief Geometric tight seal: closes the region R, the faces `input_selection` holds for,
     * and fills the gaps found with the colors `fill_selections` define.
     *
     * J is the closing of R by the disk B of diameter `distance`, minus R: the part of the rest
     * that B cannot reach. First R is made simplicially embedded (no face outside R has all its
     * vertices on R; wmtk/utils/SimplicialEmbedding.hpp): each face outside R whose three edges
     * lie on R is split at its centroid, then each edge outside R joining two vertices of R at its
     * midpoint. Then J is found in three steps.
     *  1. The dilation surface S, d(x, R) = distance/2, found on the edges that cross it (sphere
     *     traced, then bisected to the level).
     *     The mesh's boundary is no part of S, so where R + B reaches it, it acts as a wall.
     *  2. The vertices' labels and the marching (tight seal v2 note, section 2), with tol =
     *     trace_rel_tol x the mesh's bounding box diagonal (also the tolerance of both sphere
     *     traces). A vertex within distance/2 of R, R's own included, is IN if
     *     d(v, S) >= distance/2 + tol, else OUT; the others are OUT. The faces whose vertices are
     *     all IN are marked, R's included. Each edge from an IN vertex to an OUT one is split at
     *     d(x, S) = distance/2 + tol, the labels' level (sphere traced from the IN end, then
     *     bisected), and of the faces it makes, those touching its IN end are marked and the
     *     others not; the new vertex takes no label. R's faces may thus be split. J is the marked
     *     faces outside R.
     *  3. The connected components of J, through edges.
     * The colors: fill selection k (two or more) is color k; a face outside J satisfying it has
     * that color (a face of J is no color, whatever its tags). A component borders a color if one
     * of its faces shares an edge with a face of that color, and is filled if it borders
     * `min_regions` colors or more (2 or more), with those it borders. The operation throws if a
     * face sharing an edge with J satisfies several fill selections, in any component, or if a
     * face of J has a vertex on the mesh's boundary, in any component, filled or not (R more than
     * distance/2 from the boundary keeps J off it). Each component to fill (connected through
     * edges, so two touching at a vertex are filled separately) is then preprocessed, in at most
     * `max_preprocessing_rounds` rounds (0: none), split as its faces' own labels of their corners
     * require, and each resulting face takes the color its corners' labels give and gets that
     * color's tags, `fill_tags[k]` (as many as there are fill selections), added to its own
     * (tight_seal_split_fill). No topology is checked.
     *
     * `sanity`: afterwards, throw if a face is inverted, a filled face holds none of the fill tag
     * sets, or a part of the filled faces borders fewer than two colors (a concave feature of one
     * body, filled).
     */
    void tight_seal_geom(
        const ExprPtr& input_selection,
        const std::vector<ExprPtr>& fill_selections,
        const std::vector<CellTag>& fill_tags,
        double distance,
        double trace_rel_tol = 0.001,
        int64_t min_regions = 2,
        bool sanity = false,
        int64_t max_preprocessing_rounds = 10);

    void resolve_overlaps(const std::vector<std::array<ExprPtr, 2>>& intersecting_tags);

    void replace_tags(const std::vector<CellTag>& tags_in, const std::vector<CellTag>& tags_out);

    void tag_priority(const std::vector<int64_t>& tags_order);

private:
    // When set, split_edge_after binary-searches vmid onto the zero-crossing of this function.
    // Negative = stays on v1 side, positive = stays on v2 side.
    // Set before split_edge(), cleared immediately after.
    std::function<double(const Vector2d&)> m_voronoi_split_fn = nullptr;
    // The vertex the calling thread's last split made, per thread as 3D's
    // SplitTagCache::v_new: every thread of the parallel split pass writes it, and the serial
    // callers (AnnotationsTri.cpp, TightSealGeom.cpp) read their own split's. One shared size_t
    // was a write-write race, reported by ThreadSanitizer in 3 of 3 simwild 2D remeshing runs at
    // 8 threads (2026-11-05).
    wmtk::threading::enumerable_thread_specific<size_t> m_last_split_vertex;

    // Where split_face() puts the new vertex; the face's centroid if unset. Set before
    // split_face(), cleared after.
    std::optional<Vector2d> m_face_split_position;
    // split_face_before() to split_face_after(): the face's corners, attributes and edges. Face
    // splits are serial (only tight_seal_geom makes them).
    struct FaceSplitCache
    {
        std::array<size_t, 3> v;
        FaceAttributes face;
        std::array<std::pair<simplex::Edge, EdgeAttributes>, 3> edges;
        Vector2d position;
        double sizing_scalar = 1;
        size_t partition_id = 0;
    };
    FaceSplitCache m_face_split_cache;

    // tight_seal_geom's helpers that need the mesh, defined in TightSealGeom.cpp
    /// BVH of the boundary of the faces of `fids` for which in_set holds: their interfaces with
    /// the other faces and their edges on the mesh boundary. From a point outside the set, the
    /// distance to the set is the distance to this boundary. Returns false if the set is empty.
    bool tight_seal_boundary_bvh(
        const std::vector<size_t>& fids,
        const std::function<bool(size_t)>& in_set,
        SimpleBVH::BVH& bvh) const;
    /// Split edge e, keeping `faces` up to date: the children of a face in it replace it, and
    /// each child takes its parent's flag in `filled` (resized to the new capacity) and, if given,
    /// its parent's `corner_labels` for the corners it keeps (resized). Returns the new vertex, or
    /// -1 if the split was refused.
    size_t tight_seal_split_tracked(
        const simplex::Edge& e,
        std::unordered_set<size_t>& faces,
        std::vector<char>& filled,
        std::vector<CornerLabels>* corner_labels = nullptr);
    /// Split face fid at p, keeping `faces`, `filled` and `corner_labels` up to date as
    /// tight_seal_split_tracked does. Returns the new vertex, or -1 if the split was refused.
    size_t tight_seal_split_face_tracked(
        size_t fid,
        const Vector2d& p,
        std::unordered_set<size_t>& faces,
        std::vector<char>& filled,
        std::vector<CornerLabels>* corner_labels = nullptr);
    /// The dilation surface S of tight_seal_geom: the level set d(x, R) = radius, R + B's boundary,
    /// as a BVH of segments. On each edge of a face outside R (region < 0) between a vertex with
    /// dist_R_at < radius and one with dist_R_at >= radius, the crossing is where dist_R = radius:
    /// sphere traced from the near end, each step at least tol, then bisected
    /// (tight_seal::level_crossing). The crossings of each face are joined into its cut segment.
    /// Returns false if S is empty; n_traced: edges traced, n_failed: those with no crossing found
    /// (the far end is taken).
    bool tight_seal_dilation_surface_bvh(
        const std::vector<size_t>& fids,
        const std::vector<int>& region,
        const std::vector<double>& dist_R_at,
        const std::function<double(const Vector2d&)>& dist_R,
        double radius,
        double tol,
        SimpleBVH::BVH& bvh,
        size_t& n_traced,
        size_t& n_failed) const;
    /// How many of `fids` have a vertex on the mesh's boundary, a edge with no face beyond it.
    size_t tight_seal_cells_on_boundary(const std::vector<size_t>& fids) const;
    /// tight_seal_geom's fill of `components`, in 2D; as SimWildMesh::tight_seal_split_fill, with
    /// faces for tets and no faces of tets. `components_regions` gives the colors each borders
    /// (indices into `fill_tags` and `region_bvhs`); color_of gives a face's color by its tags;
    /// in_J[id] marks every face of J, whose faces are no color. First up to `max_rounds` rounds of
    /// preprocessing, each vertex labelled by the nearest of its component's colors (the first on
    /// an exact tie): a round marks each edge whose ends' labels differ if, where the distances to
    /// those two are equal (not on an end), another color of the component is nearest; then each
    /// face with three labels whose edges are all unmarked if, at its triple point inside it (where
    /// the three distances, interpolated linearly over the face, are equal), another color is
    /// nearest. Every marked face is split at its point, then every marked edge at its; the first
    /// round looks at every edge and face, each later one only at those touching a vertex the last
    /// made, until nothing is marked (or a warning after `max_rounds`). Then each face labels its
    /// corners: a corner takes its nearest color, except that the ends of an edge the face shares
    /// with a face of one of the component's colors (outside J) take that color. From these
    /// labels, all taken before any split: the faces with three labels are split at their triple
    /// point if it lies inside them (else the centroid), then the edges whose ends differ in a face
    /// holding them where the distances to the two labels are equal (not if that falls on an end).
    /// Each child keeps its parent's labels of the corners it keeps, a new vertex has none, and
    /// each face of the components then takes a color by tight_seal::split_fill_label and gets that
    /// color's fill tags added to its own. tags_before[id] gets the tags of each face it labels
    /// (filled[id] set) as they were before. No topology is checked.
    void tight_seal_split_fill(
        const std::vector<ConnectedComponent>& components,
        const std::vector<std::vector<size_t>>& components_regions,
        const std::vector<CellTag>& fill_tags,
        const std::vector<SimpleBVH::BVH>& region_bvhs,
        const std::function<int(size_t)>& color_of,
        const std::vector<char>& in_J,
        int64_t max_rounds,
        std::vector<char>& filled,
        std::vector<CellTag>& tags_before);
    /// tight_seal_geom's topology check, as SimWildMesh::tight_seal_topology_check, with faces
    /// for tets and edges for faces (components through edges).
    void tight_seal_topology_check(
        const std::vector<char>& filled,
        const std::vector<CellTag>& tags_before) const;
    /// tight_seal_geom's `sanity` checks, on the output: throws if a face is inverted, a filled
    /// face (filled[id] set) holds none of the `fill_tags` sets, or a part of the filled faces,
    /// connected through edges, borders fewer than two colors (by color_of, as in split_fill).
    void tight_seal_geom_sanity(
        const std::vector<CellTag>& fill_tags,
        const std::function<int(size_t)>& color_of,
        const std::vector<char>& filled) const;

protected:
    std::tuple<double, double> optimization_quality_stats() override;
    double optimization_stop_metric() const override { return 1.; }
    bool optimization_stop_at_float() const override { return m_sim_params.stop_at_float; }

    void write_smoothing_debug_output(const std::string& path) const override { write_vtu(path); }

    bool collapse_quality_allowed(size_t v1, size_t fid, double q, double ring_max) const override;
    bool split_adjust_position(size_t v_new, const std::vector<Tuple>& children) override;
    void split_after_vertex(size_t v_new) override { m_last_split_vertex.local() = v_new; }
    /// A face split puts the new vertex at m_face_split_position (the centroid if unset), in
    /// doubles, so every corner must be rounded. The three children take the face's attributes,
    /// the face's edges keep theirs, and the split is refused if a child is inverted.
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
};

} // namespace wmtk::components::simwild::tri
