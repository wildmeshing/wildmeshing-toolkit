#pragma once

#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/solver.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include "Parameters.h"

namespace wmtk::components::triwild {

// TODO: missing comments on what these attributes are
class VertexAttributes
{
public:
    Vector2d m_posf;
    Vector2r m_pos;
    bool m_is_rounded = false;

    bool m_is_on_surface = false;
    std::vector<int> on_bbox_faces;

    double m_sizing_scalar = 1;

    size_t partition_id = 0;

    VertexAttributes() {}
    VertexAttributes(const Vector2r& p);
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
    std::set<int64_t> tags;
    int part_id = -1;
};

class TriWildMesh : public wmtk::TriMesh
{
public:
    int m_debug_print_counter = 0;
    size_t m_tags_count = 0;
    std::map<int64_t, std::string> m_tag_id_to_name;
    std::map<std::string, int64_t> m_tag_name_to_id;

    const double MAX_ENERGY = 1e50;

    Parameters& m_params;
    std::vector<Vector2d> m_V_envelope;
    std::vector<Vector2i> m_E_envelope;
    std::shared_ptr<SampleEnvelope> m_envelope;
    double m_envelope_eps = -1;

    // for open boundary
    SampleEnvelope m_open_boundary_envelope; // todo: add sample envelope option
    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes>;
    using FaceAttCol = wmtk::AttributeCollection<FaceAttributes>;
    VertAttCol m_vertex_attribute;
    EdgeAttCol m_edge_attribute;
    FaceAttCol m_face_attribute;

    tbb::enumerable_thread_specific<std::unique_ptr<polysolve::nonlinear::Solver>> m_solver;

    // scaling factors
    double m_s_amips = -1;
    double m_s_envelope = -1;

    TriWildMesh(Parameters& _m_params, double envelope_eps, int _num_threads = 0)
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
         * eps makes it such that the energy is relative to the envelope thickness. As it's a
         * squared energy, we need eps^2.
         */
        m_s_envelope = 1. / (m_params.eps * m_params.eps);


        double& wa = m_params.w_amips;
        double& we = m_params.w_envelope;
        we = 1 - wa;
        logger().info("w_envelope = {}", we);
    }

    ~TriWildMesh() {}

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

    bool adjust_sizing_field_serial(double max_energy);

    void write_msh_groups(std::string file, const bool write_envelope = true);

    void write_vtu(const std::string& path) const;

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
     * @brief Inversion check using only floating point numbers.
     */
    bool is_inverted_f(const Tuple& loc) const;
    bool is_inverted(const std::array<size_t, 3>& vs) const;
    bool is_inverted(const Tuple& loc) const;
    double get_quality(const std::array<size_t, 3>& vs) const;
    double get_quality(const Tuple& loc) const;
    bool round(const Tuple& loc);
    //
    bool is_edge_on_surface(const Tuple& loc) const;
    bool is_edge_on_surface(const std::array<size_t, 2>& vids) const;
    bool is_edge_on_bbox(const Tuple& loc) const;
    bool is_edge_on_bbox(const std::array<size_t, 2>& vids) const;

    /**
     * @brief Check if the vertex is on the boundary of a tracked surface.
     */
    bool is_vertex_on_boundary(const size_t vid);
    //
    void mesh_improvement(int max_its = 80);

    std::tuple<double, double> local_operations(
        const std::array<int, 4>& ops,
        bool collapse_limit_length = true);
    std::tuple<double, double> get_max_avg_energy();

    /**
     * @brief Compute the winding number.
     *
     * If `vertices` and `faces` are empty, compute the winding number for the tracked surface.
     * Otherwise, compute the winding number for the input surface given by `vertices` and `faces`.
     */
    void compute_winding_number(
        const std::vector<Vector3d>& vertices = {},
        const std::vector<std::array<size_t, 3>>& faces = {});

    void compute_winding_numbers(const std::vector<std::string>& input_paths);

    void filter_with_input_surface_winding_number();
    void filter_with_tracked_surface_winding_number();
    void filter_with_flood_fill();

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
        std::set<int64_t> face_tags;
    };
    tbb::enumerable_thread_specific<SwapInfoCache> swap_cache;
};


} // namespace wmtk::components::triwild
