#pragma once

#include <wmtk/MeshData.hpp>

#include <wmtk/TriOptimizerMesh.h>

#include <wmtk/SurfaceTagAttributes.h>
#include <limits>
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
    /// The vertices of the input points (input_points), set before init: anchored there, in a
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
     * SimWildMesh::m_input_envelopes.
     *
     * Member b is built around the same curves as m_envelope, restricted to label b -- the
     * input curves of that label in generation, else the mesh's surface edges carrying it
     * (TriOptimizerMesh::init_input_envelopes()) -- at m_envelope_eps and with the predicate
     * m_envelope has. m_envelope stays the union, for the engine's direct uses of it and for a
     * surface edge carrying no label. At most 64 edge tags and triangle tags together: a mesh
     * with more throws.
     */
    void init_label_envelopes();

    /// A tracked segment's containment: its labels' envelopes if it has any, else m_envelope,
    /// as simwild 3D holds a surface face with no label. The base throws on such a segment
    /// instead; that is triwild's defect check, kept there.
    std::shared_ptr<Envelope> surface_envelope_for_edge(
        const std::array<size_t, 2>& vids,
        const EdgeAttributes& attr) const override
    {
        if (attr.m_is_surface_fs && attr.input_set() == 0) return m_envelope;
        return TriOptimizerMesh::surface_envelope_for_edge(vids, attr);
    }

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
        const std::vector<std::string>& edge_tag_names = {});

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
        const std::vector<std::string>& edge_tag_names = {});

    void init_surfaces_and_boundaries();
    /// Tracks the edges E_tagged (rows of T's vertices, as read_curves gives them) as surface,
    /// with input set bit j where E_tags has a 1 in column j. Throws on a row that is no edge.
    /// The 2D twin of SimWildMesh::init_face_tags.
    void init_edge_tags(
        const MatrixXi& E_tagged,
        const MatrixSi& E_tags,
        const std::vector<std::string>& edge_tag_names);

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

    void resolve_overlaps(const std::vector<std::array<ExprPtr, 2>>& intersecting_tags);

    void replace_tags(const std::vector<CellTag>& tags_in, const std::vector<CellTag>& tags_out);

    void tag_priority(const std::vector<int64_t>& tags_order);

private:
    // When set, split_edge_after binary-searches vmid onto the zero-crossing of this function.
    // Negative = stays on v1 side, positive = stays on v2 side.
    // Set before split_edge(), cleared immediately after.
    std::function<double(const Vector2d&)> m_voronoi_split_fn = nullptr;
    size_t m_last_split_vertex = 0;

protected:
    std::tuple<double, double> optimization_quality_stats() override;
    double optimization_stop_metric() const override { return 1.; }
    bool optimization_stop_at_float() const override { return m_sim_params.stop_at_float; }

    void write_smoothing_debug_output(const std::string& path) const override { write_vtu(path); }

    bool collapse_quality_allowed(size_t v1, size_t fid, double q, double ring_max) const override;
    bool split_adjust_position(size_t v_new, const std::vector<Tuple>& children) override;
    void split_after_vertex(size_t v_new) override { m_last_split_vertex = v_new; }
};

} // namespace wmtk::components::simwild::tri
