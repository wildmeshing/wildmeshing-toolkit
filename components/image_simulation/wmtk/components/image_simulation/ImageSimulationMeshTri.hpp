#pragma once

#include <limits>
#include <unordered_set>

#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/solver.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/write_triangle_mesh.h>
#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include "ConnectedComponent.hpp"
#include "Parameters.h"

namespace wmtk::components::image_simulation::tri {

struct VertexAttributes
{
    Vector2d m_pos;
    bool m_is_on_surface = false;
    std::vector<int> on_bbox_faces;

    double m_sizing_scalar = 1;

    size_t partition_id = 0;

    VertexAttributes() {}
    VertexAttributes(const Vector2d& p)
        : m_pos(p)
    {}
};

class EdgeAttributes
{
public:
    double tag; // TODO: is this used?

    bool m_is_surface_fs = false; // 0; 1
    /**
     * Keep track which bbox side the face is on
     * -1: none
     * 0/1: x min/max
     * 2/3: y min/max
     * 4/5: z min/max
     *
     * This bbox side ID is used to keep the bbox from collapsing.
     */
    int m_is_bbox_fs = -1; //-1; 0~5

    void reset()
    {
        m_is_surface_fs = false;
        m_is_bbox_fs = -1;
    }

    void merge(const EdgeAttributes& attr)
    {
        m_is_surface_fs = m_is_surface_fs || attr.m_is_surface_fs;
        if (attr.m_is_bbox_fs >= 0) m_is_bbox_fs = attr.m_is_bbox_fs;
    }
};

class FaceAttributes
{
public:
    double m_quality;
    double m_winding_number = 0;
    CellTag tags;
    int part_id = -1;
};

class ImageSimulationMeshTri : public wmtk::TriMesh
{
public:
    int m_debug_print_counter = 0;
    size_t m_tags_count = 0;
    std::map<int64_t, std::string> m_tag_id_to_name;
    std::map<std::string, int64_t> m_tag_name_to_id;

    const double MAX_ENERGY = std::numeric_limits<double>::max();

    Parameters& m_params;
    std::vector<Vector2d> m_V_envelope;
    std::vector<Vector2i> m_E_envelope;
    std::shared_ptr<SampleEnvelope> m_envelope;
    std::shared_ptr<SampleEnvelope> m_envelope_orig;
    double m_envelope_eps = -1;

    using VertAttCol = AttributeCollection<VertexAttributes>;
    using EdgeAttCol = AttributeCollection<EdgeAttributes>;
    using FaceAttCol = AttributeCollection<FaceAttributes>;
    VertAttCol m_vertex_attribute;
    EdgeAttCol m_edge_attribute;
    FaceAttCol m_face_attribute;

    bool m_collapse_check_link_condition = false; // classical link condition
    bool m_collapse_check_topology = false; // sanity check
    bool m_collapse_check_manifold = false; // manifoldness check after collapse

    tbb::enumerable_thread_specific<std::unique_ptr<polysolve::nonlinear::Solver>> m_solver;
    std::vector<double> m_surface_mass; // the mass matrix for surface vertices
    std::vector<Vector3d> m_surface_stiffness; // stiffness matrix for surface vertices

    // scaling factors
    double m_s_amips = -1;
    double m_s_smooth = -1;
    double m_s_envelope = -1;
    double m_s_barrier = -1;

    ImageSimulationMeshTri(Parameters& _m_params, double envelope_eps, int _num_threads = 0)
        : m_params(_m_params)
        , m_envelope_eps(envelope_eps)
    {
        NUM_THREADS = _num_threads;
        p_vertex_attrs = &m_vertex_attribute;
        p_edge_attrs = &m_edge_attribute;
        p_face_attrs = &m_face_attribute;

        optimization::deactivate_opt_logger();

        m_s_amips = 1.;
        /**
         * The bilaplacian energy for smoothing in 2D scales inverse to the length, because the
         * Laplace operator times the positions scales inverse to the length. The mass scales with
         * the length but as the bilaplacian contains twice the Laplace operator times positions but
         * only once the mass, the entire energy scales inverse to the length.
         *
         * In 3D, the mass scales to length^2 and therefore the energy is dimensionless. That is not
         * the case in 2D.
         */
        m_s_smooth = m_params.diag_l;
        /**
         * The diagonal compensates for the mass (in 2D its just a length, in 3D its an
         * area and we need the squared diagonal).
         * eps makes it such that the energy is relative to the envelope thickness. As it's a
         * squared energy, we need eps^2.
         */
        m_s_envelope = 1. / (m_params.diag_l * m_params.eps * m_params.eps);
        /**
         * The barrier energy computes a double-sided squared distance and therefore needs to be
         * scaled by length^4.
         */
        m_s_barrier = 1. / (m_params.diag_l4);

        // check weights (ignoring barrier here)
        {
            double& wa = m_params.w_amips;
            double& ws = m_params.w_smooth;
            const double sum = wa + ws;
            if (sum > 1) {
                wa /= sum;
                ws /= sum;
                logger().warn(
                    "Weights for AMIPS and smooth sum up to greater than 1. Rescaling to \n  "
                    "w_amips = {}, \n  w_smooth = {}",
                    wa,
                    ws);
            }
            double& we = m_params.w_envelope;
            we = 1 - (wa + ws);
            logger().info("w_envelope = {}", we);
        }

        init_separation_weight();
    }

    ~ImageSimulationMeshTri() {}

    // TODO: this should not be here
    void partition_mesh();

    // TODO: morton should not be here, but inside wmtk
    void partition_mesh_morton();

    size_t get_partition_id(const Tuple& loc) const
    {
        return m_vertex_attribute[loc.vid(*this)].partition_id;
    }

    double get_length2(const Tuple& l) const;


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

    void init_surfaces_and_boundaries();

    void init_envelope(const MatrixXd& V, const MatrixXi& F);

    void init_separation_weight();

    CellTag string_set_to_cell_tag(const std::set<std::string>& str_set);

    bool adjust_sizing_field_serial(double max_energy);

    void write_msh(std::string file, const bool write_envelope = true);
    void write_msh_groups(std::string file, const bool write_envelope = true);

    void write_vtu(const std::string& path) const;
    void write_vtu_with_energies(const std::string& path) const;

    std::vector<std::array<size_t, 2>> get_edges_by_condition(
        std::function<bool(const EdgeAttributes&)> cond) const;

public:
    void split_all_edges();
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& loc) override;

    void collapse_all_edges(bool is_limit_length = true);
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    size_t swap_all_edges();
    /**
     * @brief The quality improvement of a swap.
     *
     * Used to determine the priority and weight of a swap operation.
     */
    double swap_weight(const Tuple& t) const;
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    void smooth_all_vertices(const size_t n_iters = 1);
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    void build_mass_matrix();
    /**
     * @brief A vector containing the vertex position and all positions of the surface neighbors.
     *
     * Returns an empty vector if vertex is not on the surface.
     */
    std::vector<Vector2d> get_surface_assembles(const Tuple& t) const;
    std::shared_ptr<polysolve::nonlinear::Problem> get_smooth_energy(const Tuple& t) const;
    std::shared_ptr<polysolve::nonlinear::Problem> get_envelope_energy(const Tuple& t) const;
    /**
     * @brief Get the energy for minimal separation.
     *
     * By default, this method constructs the energy only using a local neighborhood. Using the
     * entire surface is extremely slow.
     */
    std::shared_ptr<polysolve::nonlinear::Problem> get_barrier_energy(
        const Tuple& t,
        const bool use_full_surface = false) const;

    std::vector<std::array<double, 6>> get_amips_assembles(const Tuple& t) const;
    std::shared_ptr<polysolve::nonlinear::Problem> get_amips_energy(const Tuple& t) const;

    /**
     * For debugging purposes.
     */
    void log_total_surface_energy();
    //
    /**
     * @brief Inversion check using only floating point numbers.
     */
    bool is_inverted(const std::array<size_t, 3>& vs) const;
    bool is_inverted(const Tuple& loc) const;
    bool is_inverted(const size_t fid) const;
    double get_quality(const std::array<size_t, 3>& vs) const;
    double get_quality(const Tuple& loc) const;
    double get_quality(const size_t fid) const;

    double triangle_area(const size_t fid) const;

    //
    bool is_edge_on_surface(const Tuple& loc) const;
    bool is_edge_on_surface(const std::array<size_t, 2>& vids) const;
    bool is_edge_on_bbox(const Tuple& loc) const;
    bool is_edge_on_bbox(const std::array<size_t, 2>& vids) const;
    //
    void mesh_improvement(int max_its = 80);
    std::tuple<double, double> local_operations(
        const std::array<int, 4>& ops,
        bool collapse_limit_length = true);
    std::tuple<double, double> get_max_avg_energy();

    /**
     * @brief Find all connected components that contain the `tag_in` tags.
     */
    std::vector<ConnectedComponent> compute_connected_components(const CellTag& tag_in) const;

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

    void resolve_intersections(const std::vector<CellTag>& intersecting_tags);

    void replace_tags(const std::vector<CellTag>& tags_in, const CellTag& tag_out);

    /**
     * @brief Find the substructure region that might be affected by minimal separation.
     *
     * The region considers all edges within 1.5 * (dhat + l_max), where l_max is the maximum
     * incident edge length to the vertex and dhat is given by the parameters.
     *
     * @param v_tuple The vertex the region should be found for.
     * @param V Nx2 vertex positions in the region
     * @param E Nx2 edges in the region
     */
    void substructure_region(const Tuple& v_tuple, MatrixXd& V, MatrixXi& E, size_t& vid) const;

    bool vertex_is_on_surface(const size_t vid) const override
    {
        return m_vertex_attribute.at(vid).m_is_on_surface ||
               !m_vertex_attribute.at(vid).on_bbox_faces.empty();
    }
    bool edge_is_on_surface(const std::array<size_t, 2>& vids) const override
    {
        if (!vertex_is_on_surface(vids[0]) || !vertex_is_on_surface(vids[1])) {
            return false;
        }

        const auto [_, eid] = tuple_from_edge(vids);
        bool on_surface = m_edge_attribute.at(eid).m_is_surface_fs;
        bool on_bbox = m_edge_attribute.at(eid).m_is_bbox_fs >= 0;
        return on_surface || on_bbox;
    }

private:
    ////// Operations

    struct SplitInfoCache
    {
        //        VertexAttributes vertex_info;
        size_t v1_id;
        size_t v2_id;
        std::vector<size_t> v1_param_type;
        std::vector<size_t> v2_param_type;

        EdgeAttributes old_e_attrs;

        // std::vector<std::pair<EdgeAttributes, std::array<size_t, 2>>> changed_edges;
        std::map<simplex::Edge, EdgeAttributes> changed_edges;

        /**
         * All faces incident to the splitted edge, identified by the link vertex (the vertex
         * opposite to the splitted edge).
         */
        std::map<size_t, FaceAttributes> faces;
    };
    tbb::enumerable_thread_specific<SplitInfoCache> split_cache;

    struct CollapseInfoCache
    {
        size_t v1_id;
        size_t v2_id;
        double max_energy;
        double edge_length;
        bool is_limit_length;

        std::vector<std::pair<EdgeAttributes, std::array<size_t, 2>>> changed_edges;
        // all faces incident to the delete vertex (v1) that are on the tracked surface
        std::vector<std::array<size_t, 2>> surface_edges;
        std::vector<size_t> changed_fids;
        std::vector<double> changed_energies;
    };
    tbb::enumerable_thread_specific<CollapseInfoCache> collapse_cache;


    struct SwapInfoCache
    {
        double max_energy;
        std::map<simplex::Edge, EdgeAttributes> changed_edges;
        CellTag face_tags;
    };
    tbb::enumerable_thread_specific<SwapInfoCache> swap_cache;

    // When set, split_edge_after binary-searches vmid onto the zero-crossing of this function.
    // Negative = stays on v1 side, positive = stays on v2 side.
    // Set before split_edge(), cleared immediately after.
    std::function<double(const Vector2d&)> m_voronoi_split_fn = nullptr;
};

} // namespace wmtk::components::image_simulation::tri
