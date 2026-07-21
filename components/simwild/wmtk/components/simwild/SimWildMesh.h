#pragma once

#include <igl/Timer.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Morton.h>
#include <wmtk/utils/PartitionMesh.h>
#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/threading/concurrent_map.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <wmtk/threading/parallel_for.hpp>
#include <wmtk/threading/parallel_sort.hpp>

#include "ConnectedComponent.hpp"
#include "Parameters.h"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <VolumeRemesher/embed.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <igl/remove_unreferenced.h>
#include <memory>

#include "expression_parser/Expression.hpp"

namespace wmtk::components::simwild {

class VertexAttributes
{
public:
    Vector3r m_pos; // exact position in rational
    Vector3d m_posf; // position as double
    /**
     * If a vertex cannot be rounded without inverting a tet, the exact position must be used. Once
     * the vertex can be rounded to double precision, the rational representation is obsolete.
     */
    bool m_is_rounded = false;

    bool m_is_on_surface = false;
    /**
     * The order of a vertex in a TetMesh is as follows:
     * 0: vertex is not on the surface
     * 1: vertex is on the surface
     * 2: vertex is on the boundary of the surface or a non-manifold edge
     * 3: vertex is at the boundary of a non-manifold edge or a non-manifold vertex
     */
    size_t m_order = 0;
    std::vector<int> on_bbox_faces; // same as is_bbox_fs?

    double m_sizing_scalar = 1;

    /**
     * Required for multi-threading.
     */
    size_t partition_id = 0;

    VertexAttributes() {};
    VertexAttributes(const Vector3r& p);
};


// class EdgeAttributes
// {
// public:
//     bool m_is_on_open_boundary = false;
// };

class FaceAttributes
{
public:
    /**
     * Is this face a part of the surface.
     */
    bool m_is_surface_fs = false;

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

    void merge(const FaceAttributes& attr)
    {
        m_is_surface_fs = m_is_surface_fs || attr.m_is_surface_fs;
        if (attr.m_is_bbox_fs >= 0) m_is_bbox_fs = attr.m_is_bbox_fs;
    }
};

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

class SimWildMesh : public wmtk::TetMesh
{
public:
    using ExprPtr = expression_parser::ExpressionPtr;

    int m_debug_print_counter = 0;
    size_t m_tags_count = 0;
    std::map<int64_t, std::string> m_tag_id_to_name;
    std::map<std::string, int64_t> m_tag_name_to_id;

    double time_env = 0.0;
    igl::Timer isout_timer;
    const double MAX_ENERGY = std::numeric_limits<double>::max();

    Parameters& m_params;
    std::vector<Vector3d> m_V_envelope;
    std::vector<Vector3i> m_F_envelope;
    std::shared_ptr<SampleEnvelope> m_envelope;
    std::shared_ptr<SampleEnvelope> m_envelope_orig;
    double m_envelope_eps = -1;

    std::vector<std::tuple<ExprPtr, double>> m_sizing_field;

    bool m_collapse_check_quality = true;

    // for open boundary
    std::shared_ptr<SampleEnvelope> m_order_2_edge_envelope; // todo: add sample envelope option

    wmtk::threading::enumerable_thread_specific<std::unique_ptr<polysolve::nonlinear::Solver>>
        m_solver;

    // scaling factors
    double m_s_amips = -1;
    double m_s_envelope = -1;

    // When set, split_edge_after binary-searches vmid onto the zero-crossing of this function.
    // Negative = stays on v1 side, positive = stays on v2 side.
    std::function<double(const Vector3d&)> m_voronoi_split_fn = nullptr;

    SimWildMesh(Parameters& _m_params, double envelope_eps, int _num_threads = 0)
        : m_params(_m_params)
        , m_envelope_eps(envelope_eps)
    {
        NUM_THREADS = _num_threads;
        p_vertex_attrs = &m_vertex_attribute;
        p_face_attrs = &m_face_attribute;
        p_tet_attrs = &m_tet_attribute;
        m_collapse_check_link_condition = false;
        m_collapse_check_manifold = false;

        // solver is lazily created on first use

        optimization::deactivate_opt_logger();

        m_s_amips = 1.;
        m_s_envelope = 1. / (m_params.diag_l * m_params.eps * m_params.eps);

        double& wa = m_params.w_amips;
        double& we = m_params.w_envelope;
        we = 1 - wa;
        logger().info("w_envelope = {}", we);
    }

    ~SimWildMesh() {}
    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    using FaceAttCol = wmtk::AttributeCollection<FaceAttributes>;
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    // using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes>;
    VertAttCol m_vertex_attribute;
    FaceAttCol m_face_attribute;
    TetAttCol m_tet_attribute;
    // EdgeAttCol m_edge_attribute;

    // only used with unit tests
    void create_mesh_attributes(
        const std::vector<VertexAttributes>& _vertex_attribute,
        const std::vector<TetAttributes>& _tet_attribute)
    {
        auto n_tet = _tet_attribute.size();
        m_vertex_attribute.resize(_vertex_attribute.size());
        m_face_attribute.resize(4 * n_tet);
        m_tet_attribute.resize(n_tet);

        for (auto i = 0; i < _vertex_attribute.size(); i++) {
            m_vertex_attribute[i] = _vertex_attribute[i];
        }
        m_tet_attribute.m_attributes = std::vector<TetAttributes>(_tet_attribute.size());
        for (auto i = 0; i < _tet_attribute.size(); i++) {
            m_tet_attribute[i] = _tet_attribute[i];
        }
        for (auto i = 0; i < _tet_attribute.size(); i++) {
            m_tet_attribute[i].m_quality = get_quality(tuple_from_tet(i));
        }
    }

    // TODO This should not be here but inside wmtk
    void compute_vertex_partition()
    {
        auto partition_id = partition_TetMesh(*this, NUM_THREADS);
        for (auto i = 0; i < vert_capacity(); i++)
            m_vertex_attribute[i].partition_id = partition_id[i];
    }

    // TODO This should not be here but inside wmtk
    void compute_vertex_partition_morton()
    {
        if (NUM_THREADS == 0) {
            return;
        }

        logger().info("Number of parts: {} by morton", NUM_THREADS);

        std::vector<Eigen::Vector3d> V_v(vert_capacity());

        wmtk::threading::parallel_for(
            wmtk::threading::blocked_range<int>(0, V_v.size()),
            [&](wmtk::threading::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    V_v[i] = m_vertex_attribute[i].m_posf;
                }
            },
            NUM_THREADS);


        struct sortstruct
        {
            int order;
            Resorting::MortonCode64 morton;
        };

        std::vector<sortstruct> list_v;
        list_v.resize(V_v.size());
        // since the morton code requires a correct scale of input vertices,
        //  we need to scale the vertices if their coordinates are out of range
        std::vector<Eigen::Vector3d> V = V_v; // this is for rescaling vertices
        Eigen::Vector3d vmin, vmax;
        vmin = V.front();
        vmax = V.front();

        for (size_t j = 0; j < V.size(); j++) {
            for (int i = 0; i < 3; i++) {
                vmin(i) = std::min(vmin(i), V[j](i));
                vmax(i) = std::max(vmax(i), V[j](i));
            }
        }

        // get_bb_corners(V, vmin, vmax);
        Eigen::Vector3d center = (vmin + vmax) / 2;

        wmtk::threading::parallel_for(
            wmtk::threading::blocked_range<int>(0, V.size()),
            [&](wmtk::threading::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    V[i] = V[i] - center;
                }
            },
            NUM_THREADS);

        Eigen::Vector3d scale_point =
            vmax - center; // after placing box at origin, vmax and vmin are symetric.

        double xscale, yscale, zscale;
        xscale = fabs(scale_point[0]);
        yscale = fabs(scale_point[1]);
        zscale = fabs(scale_point[2]);
        double scale = std::max(std::max(xscale, yscale), zscale);
        if (scale > 300) {
            wmtk::threading::parallel_for(
                wmtk::threading::blocked_range<int>(0, V.size()),
                [&](wmtk::threading::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        V[i] = V[i] / scale;
                    }
                },
                NUM_THREADS);
        }

        constexpr int multi = 1000;
        wmtk::threading::parallel_for(
            wmtk::threading::blocked_range<int>(0, V.size()),
            [&](wmtk::threading::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    list_v[i].morton = Resorting::MortonCode64(
                        int(V[i][0] * multi),
                        int(V[i][1] * multi),
                        int(V[i][2] * multi));
                    list_v[i].order = i;
                }
            },
            NUM_THREADS);

        const auto morton_compare = [](const sortstruct& a, const sortstruct& b) {
            return (a.morton < b.morton);
        };

        wmtk::threading::parallel_sort(list_v.begin(), list_v.end(), morton_compare);

        int interval = list_v.size() / NUM_THREADS + 1;

        wmtk::threading::parallel_for(
            wmtk::threading::blocked_range<int>(0, list_v.size()),
            [&](wmtk::threading::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    m_vertex_attribute[list_v[i].order].partition_id = i / interval;
                }
            },
            NUM_THREADS);
    }

    size_t get_partition_id(const Tuple& loc) const
    {
        return m_vertex_attribute[loc.vid(*this)].partition_id;
    }

    void init_envelope(const MatrixXd& V, const MatrixXi& F, const bool use_exact);

    CellTag string_set_to_cell_tag(const std::set<std::string>& str_set);

    void set_sizing_field(const nlohmann::json& m_sizing_field_json);

    double get_length2(const Tuple& l) const;

    ////// Attributes related

    void write_msh(std::string file, const bool write_envelope = true);
    void output_faces(std::string file, std::function<bool(const FaceAttributes&)> cond);

public:
    void split_all_edges();
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& loc) override;

    void smooth_all_vertices(const size_t n_iters);
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    void collapse_all_edges(bool is_limit_length = true);
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    void simplify();

    size_t swap_all_edges_44();
    bool swap_edge_44_before(const Tuple& t) override;
    double swap_edge_44_energy(const std::vector<std::array<size_t, 4>>& tets, const int op_case)
        override;
    bool swap_edge_44_after(const Tuple& t) override;

    size_t swap_all_edges_56();
    bool swap_edge_56_before(const Tuple& t) override;
    double swap_edge_56_energy(const std::vector<std::array<size_t, 4>>& tets, const int op_case)
        override;
    bool swap_edge_56_after(const Tuple& t) override;

    size_t swap_all_edges_32();
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    size_t swap_all_faces();
    bool swap_face_before(const Tuple& t) override;
    bool swap_face_after(const Tuple& t) override;

    size_t swap_all_edges_all();

    /**
     * @brief Inversion check using only floating point numbers.
     */
    bool is_inverted_f(const Tuple& loc) const;
    bool is_inverted(const std::array<size_t, 4>& vs) const;
    bool is_inverted(const Tuple& loc) const;
    double get_quality(const std::array<size_t, 4>& vs) const;
    double get_quality(const Tuple& loc) const;

    std::vector<std::array<double, 12>> get_amips_assembles(const Tuple& t) const;

    std::shared_ptr<polysolve::nonlinear::Problem> get_amips_energy(const Tuple& t) const;
    std::shared_ptr<polysolve::nonlinear::Problem> get_envelope_energy(const Tuple& t) const;

    /**
     * @brief Round a vertex position to floating point.
     *
     * Only rounds the vertex position, if it does not cause inverted elements.
     *
     * @return True if successful or already rounded, false otherwise.
     */
    bool round(const Tuple& loc);

    /**
     * @brief Check if all vertices of the mesh are rounded.
     *
     */
    bool all_rounded() const;

    //
    bool is_edge_on_surface(const Tuple& loc);
    bool is_edge_on_bbox(const Tuple& loc);
    //
    void mesh_improvement(int max_its = 80);
    std::tuple<double, double> local_operations(
        const std::array<int, 4>& ops,
        bool collapse_limit_length = true);
    std::tuple<double, double> get_max_avg_energy();

    bool check_attributes();

    std::vector<std::array<size_t, 3>> get_faces_by_condition(
        std::function<bool(const FaceAttributes&)> cond) const;

    bool invariants(const std::vector<Tuple>& t) override; // this is now automatically checked

    // debug use
    std::atomic<int> cnt_split = 0, cnt_collapse = 0, cnt_swap = 0;

private:
    ////// Operations

    struct SplitInfoCache
    {
        //        VertexAttributes vertex_info;
        size_t v_new;
        size_t v1_id;
        size_t v2_id;
        bool is_edge_on_surface = false;
        bool is_edge_open_boundary = false;
        std::vector<size_t> v1_param_type;
        std::vector<size_t> v2_param_type;

        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;

        /**
         * All tets incident to the splitted edge, identified by the link edge (the edge opposite to
         * the splitted one).
         */
        std::map<simplex::Edge, TetAttributes> tets;
    };
    wmtk::threading::enumerable_thread_specific<SplitInfoCache> split_cache;

    struct CollapseInfoCache
    {
        size_t v1_id;
        size_t v2_id;
        double max_energy;
        double edge_length;
        bool is_limit_length;

        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;
        // all faces incident to the delete vertex (v1) that are on the tracked surface
        std::vector<std::array<size_t, 3>> surface_faces;
        // all edges incident to the deleted vertex(v1) that are on the open boundary
        std::vector<std::array<size_t, 2>> boundary_edges;
        std::vector<size_t> changed_tids;
        std::vector<double> changed_energies;
    };
    wmtk::threading::enumerable_thread_specific<CollapseInfoCache> collapse_cache;


    struct SwapInfoCache
    {
        double max_energy;
        std::map<std::array<size_t, 3>, FaceAttributes> changed_faces;
        CellTag tet_tags;
    };
    wmtk::threading::enumerable_thread_specific<SwapInfoCache> swap_cache;

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
        const std::vector<std::string>& tag_names);
    void init_from_image(
        const MatrixXd& V,
        const MatrixXi& T,
        const MatrixSi& T_tags,
        const std::vector<std::string>& tag_names);

    void init_surfaces_and_boundaries();

    std::vector<std::array<size_t, 3>> triangulate_polygon_face(std::vector<Vector3r> points);

    bool adjust_sizing_field_serial(double max_energy);

    /**
     * @brief Find open boundary edges of the embedded surface and initialize a BVH for the open
     * boundary.
     *
     * The envelope for the open boundary uses a hack: A boundary edge is represented as a
     * degenerate triangle, e.g., (v0,v1,v0). That way, the standard triangle envelope code can be
     * used.
     *
     */
    void find_order_2_edges();
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

    void write_vtu(const std::string& path);

    void write_surface(const std::string& path) const;

public:
    // substructure functions

    bool vertex_is_on_surface(const size_t vid) const override;

    bool face_is_on_surface(const size_t fid) const override;

    size_t get_order_of_vertex(const size_t vid) const override;
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
