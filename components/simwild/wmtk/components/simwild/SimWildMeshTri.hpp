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

    std::vector<std::tuple<ExprPtr, double>> m_sizing_field;
    std::vector<std::tuple<ExprPtr, double>> m_quality_field;

    bool m_collapse_check_link_condition = false; // classical link condition
    bool m_collapse_check_topology = false; // sanity check
    bool m_collapse_check_manifold = false; // manifoldness check after collapse

    /// No 0-dimensional features here, so smoothing is never positionally constrained beyond
    /// the envelope. See TriOptimizerMesh's feature-point anchors for the case that is.
    bool smoothing_position_is_allowed(const size_t, const Vector2d&) const override
    {
        return true;
    }

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
        const std::vector<std::string>& tag_names);

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
        const std::vector<std::string>& tag_names);

    void init_surfaces_and_boundaries();

    void init_envelope(const MatrixXd& V, const MatrixXi& F);

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


    void write_vtu(const std::string& path, MeshMap* output = nullptr) const;
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
     * @brief Geometric tight seal between the regions R_i, the faces tagged with `tags[i]` and
     * nothing else.
     *
     * J is the closing of the union R of the R_i by the disk B of diameter `distance`, minus R:
     * the part of E that B cannot reach. First R is made simplicially embedded (no face outside R
     * has all its vertices on R; wmtk/utils/SimplicialEmbedding.hpp): each face outside R whose
     * three edges lie on R is split at its centroid, then each edge outside R joining two vertices
     * of R at its midpoint. Without it, a gap one face wide would be in J at any distance, every
     * vertex of R counting as inside. Then J is found in three steps.
     *  1. The dilation surface S, d(x, R) = distance/2, sphere traced on the edges that cross it.
     *     The mesh's boundary is no part of S, so where R + B reaches it, it acts as a wall.
     *  2. A vertex is inside if it is in R + B and d(v, S) > distance/2, counted from
     *     (1 - trace_rel_tol) x distance/2 up, so a vertex on the level set is never split off
     *     by rounding; every vertex of R is inside (R lies in the closing; S, linear in each
     *     face, can pass closer to R than the level set). Every face outside R with a vertex
     *     inside is processed: each of its edges from an inside vertex to an outside one is
     *     split where it leaves J, at d(x, S) = distance/2, sphere traced from the inside end
     *     (to within trace_rel_tol x distance/2; not split if the inside end is already that
     *     close or closer), and J is the processed faces whose vertices are all inside.
     *  3. The connected components of J (through edges) are kept by the rule of `selection`.
     * The operation throws if a face of J has a vertex on the mesh's boundary, in any component,
     * filled or not. Regions more than distance/2 from the boundary keep J off it.
     * Each component (connected through edges, so two touching at a vertex are filled
     * separately) is then filled with the Voronoi diagram of its regions, by enumeration
     * (MultiVoronoiEnumeration.hpp): every vertex gets the set of regions nearest to it, each
     * face is split as its corners' sets require (an edge where its ends' nearest regions
     * change, twice where a third region crosses it; the face itself at its centroid where three
     * regions meet inside it), and each face gets the tag of the region its corners share.
     *
     * `selection` "explicit": the regions are `tags`, two or three, which may not hold ambient
     * (the tag named "ambient"). "detected": `tags` is ignored (with a warning if it is not
     * empty), the regions are every tag but ambient that some face has alone, and each part that
     * borders `min_regions` (2 or 3; ignored, with a warning if not 2, for "explicit") to three of
     * them is filled with those it borders. A part bordering four or more is left alone, with a
     * warning: the fill handles at most three regions.
     *
     * `mode` is EXPERIMENTAL. "voronoi": as above. "fill": the limit of three regions goes: any
     * number of `tags` from two up, any `min_regions` from two up, and a component may border any
     * number of regions. Its faces are not split: each takes the nearest of the regions the
     * component borders, and each region grows over those that took it, as long as that keeps
     * its topology (tight_seal::topology_fill). A face left out throws.
     *
     * `sanity`: afterwards, throw if a face is inverted, a filled face carries other than one
     * region's tag, or a part of the filled faces borders fewer than two regions (a concave
     * feature of one body, filled).
     */
    void tight_seal_geom(
        const std::vector<int64_t>& tags,
        double distance,
        double trace_rel_tol = 0.01,
        const std::string& selection = "explicit",
        int64_t min_regions = 2,
        bool sanity = false,
        const std::string& mode = "voronoi");

    void resolve_overlaps(const std::vector<std::array<ExprPtr, 2>>& intersecting_tags);

    void replace_tags(const std::vector<CellTag>& tags_in, const std::vector<CellTag>& tags_out);

    void tag_priority(const std::vector<int64_t>& tags_order);

private:
    // When set, split_edge_after binary-searches vmid onto the zero-crossing of this function.
    // Negative = stays on v1 side, positive = stays on v2 side.
    // Set before split_edge(), cleared immediately after.
    std::function<double(const Vector2d&)> m_voronoi_split_fn = nullptr;
    size_t m_last_split_vertex = 0;

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
    /// each child takes its parent's flag in `filled` (resized to the new capacity). Calls
    /// on_child(id, end_vid) for each child in `faces`, end_vid being the endpoint of e the child
    /// keeps. Returns the new vertex, or -1 if the split was refused.
    size_t tight_seal_split_tracked(
        const simplex::Edge& e,
        std::unordered_set<size_t>& faces,
        std::vector<char>& filled,
        const std::function<void(size_t, size_t)>& on_child);
    /// Split face fid at p, keeping `faces` and `filled` up to date as tight_seal_split_tracked
    /// does. Returns the new vertex, or -1 if the split was refused.
    size_t tight_seal_split_face_tracked(
        size_t fid,
        const Vector2d& p,
        std::unordered_set<size_t>& faces,
        std::vector<char>& filled);
    /// The dilation surface S of tight_seal_geom: the level set d(x, R) = radius, R + B's boundary,
    /// as a BVH of segments. On each edge of a face outside R (region < 0) between a vertex with
    /// dist_R_at < radius and one with dist_R_at >= radius, the crossing is sphere traced on dist_R
    /// from the near end (to within tol, on R's side); the crossings of each face are joined into
    /// its cut segment. Returns false if S is empty; n_traced: edges traced, n_failed: traces that left
    /// their edge (the far end is taken).
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
    /// tight_seal_geom's "fill" mode: labels the cells of `components` and grows each region over
    /// them, keeping its topology (tight_seal::topology_fill). Throws if a face is left.
    void tight_seal_topology_fill(
        const std::vector<ConnectedComponent>& components,
        const std::vector<std::vector<size_t>>& components_regions,
        const std::vector<int64_t>& tags,
        const std::vector<SimpleBVH::BVH>& region_bvhs,
        std::vector<char>& filled);
    /// tight_seal_geom's `sanity` checks, on the output: throws if a face is inverted, a filled
    /// face (filled[id] set) carries other than exactly one of `tags`, or a part of the filled
    /// faces, connected through edges, borders fewer than two of them.
    void tight_seal_geom_sanity(const std::vector<int64_t>& tags, const std::vector<char>& filled)
        const;

protected:
    std::tuple<double, double> optimization_quality_stats() override;
    double optimization_stop_metric() const override { return 1.; }
    bool optimization_stop_at_float() const override { return m_sim_params.stop_at_float; }

    void write_smoothing_debug_output(const std::string& path) const override { write_vtu(path); }

    bool collapse_quality_allowed(size_t v1, size_t fid, double q, double ring_max) const override;
    bool split_adjust_position(size_t v_new, const std::vector<Tuple>& children) override;
    void split_after_vertex(size_t v_new) override { m_last_split_vertex = v_new; }
    /// A face split puts the new vertex at m_face_split_position (the centroid if unset), in
    /// doubles, so every corner must be rounded. The three children take the face's attributes,
    /// the face's edges keep theirs, and the split is refused if a child is inverted.
    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;
};

} // namespace wmtk::components::simwild::tri
