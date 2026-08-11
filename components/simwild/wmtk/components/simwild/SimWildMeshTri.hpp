#pragma once

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

    std::vector<std::tuple<ExprPtr, double>> m_sizing_field;
    std::vector<std::tuple<ExprPtr, double>> m_quality_field;

    bool m_collapse_check_link_condition = false; // classical link condition
    bool m_collapse_check_topology = false; // sanity check
    bool m_collapse_check_manifold = false; // manifoldness check after collapse

    /// No 0-dimensional features here, so smoothing is never positionally constrained beyond
    /// the envelope. See TriWildMesh::smoothing_position_is_allowed for the case that is.
    bool smoothing_position_is_allowed(const size_t, const Vector2d&) const override { return true; }

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
    double quality_rel(const size_t tid) const;
    double quality_rel(const Tuple& t) const;
    bool check_mesh_quality(double& max_rel_quality, const bool verbose = false) const;
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
    size_t refine_sizing_around_worst();

    void write_msh(std::string file, const bool write_envelope = true);

    void write_vtu(const std::string& path) const;
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

    void mesh_improvement(int max_its = 80);
    double local_operations(const std::array<int, 4>& ops, bool collapse_limit_length = true);

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
    void write_smoothing_debug_output(const std::string& path) const override { write_vtu(path); }

    bool collapse_quality_allowed(size_t v1, size_t fid, double q, double ring_max)
        const override;
    void collapse_after_vertex(size_t v1, size_t v2) override;
    bool split_adjust_position(size_t v_new, const std::vector<Tuple>& children) override;
    void split_after_vertex(size_t v_new) override { m_last_split_vertex = v_new; }
};

} // namespace wmtk::components::simwild::tri
