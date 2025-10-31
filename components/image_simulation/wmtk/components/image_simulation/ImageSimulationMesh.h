#pragma once

#include <igl/Timer.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Morton.h>
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/simplex/RawSimplex.hpp>
#include "Parameters.h"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/concurrent_map.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#include <tbb/parallel_sort.h>
#include <wmtk/utils/EnableWarnings.hpp>
#include <VolumeRemesher/embed.h>
// clang-format on

#include <igl/remove_unreferenced.h>
#include <memory>

namespace wmtk::components::image_simulation {

// TODO: missing comments on what these attributes are
class VertexAttributes
{
public:
    Vector3r m_pos;
    Vector3d m_posf;
    bool m_is_rounded = false;

    bool m_is_on_surface = false;
    std::vector<int> on_bbox_faces; // same as is_bbox_fs?

    double m_sizing_scalar = 1;

    size_t partition_id = 0;

    // for open boundary
    bool m_is_on_open_boundary = false;

    VertexAttributes() {};
    VertexAttributes(const Vector3r& p);
};


// class EdgeAttributes
// {
// public:
//     bool m_is_on_open_boundary = false;
// };

// TODO: missing comments on what these attributes are
class FaceAttributes
{
public:
    double tag;

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

    void merge(const FaceAttributes& attr)
    {
        m_is_surface_fs = m_is_surface_fs || attr.m_is_surface_fs;
        if (attr.m_is_bbox_fs >= 0) m_is_bbox_fs = attr.m_is_bbox_fs;
    }
};

// TODO: missing comments on what these attributes are
class TetAttributes
{
public:
    double m_quality;
    double m_winding_number = 0;
    int64_t tag = 0;
    int part_id = -1;
};

class ImageSimulationMesh : public wmtk::TetMesh
{
public:
    int m_debug_print_counter = 0;

    double time_env = 0.0;
    igl::Timer isout_timer;
    const double MAX_ENERGY = std::numeric_limits<double>::max();

    Parameters& m_params;
    std::shared_ptr<Envelope> m_envelope;
    // for surface projection
    std::shared_ptr<SampleEnvelope> triangles_tree;
    double m_envelope_eps = -1;

    // for open boundary
    wmtk::ExactEnvelope m_open_boundary_envelope; // todo: add sample envelope option
    SampleEnvelope boundaries_tree;

    ImageSimulationMesh(Parameters& _m_params, double envelope_eps, int _num_threads = 0)
        : m_params(_m_params)
        , m_envelope_eps(envelope_eps)
    {
        NUM_THREADS = _num_threads;
        p_vertex_attrs = &m_vertex_attribute;
        p_face_attrs = &m_face_attribute;
        p_tet_attrs = &m_tet_attribute;
        m_collapse_check_link_condition = false;
    }

    ~ImageSimulationMesh() {}
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
        m_tet_attribute.m_attributes = tbb::concurrent_vector<TetAttributes>(_tet_attribute.size());
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
        if (NUM_THREADS == 0) return;

        wmtk::logger().info("Number of parts: {} by morton", NUM_THREADS);

        tbb::task_arena arena(NUM_THREADS);

        arena.execute([&] {
            std::vector<Eigen::Vector3d> V_v(vert_capacity());

            tbb::parallel_for(
                tbb::blocked_range<int>(0, V_v.size()),
                [&](tbb::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        V_v[i] = m_vertex_attribute[i].m_posf;
                    }
                });


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

            tbb::parallel_for(tbb::blocked_range<int>(0, V.size()), [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    V[i] = V[i] - center;
                }
            });

            Eigen::Vector3d scale_point =
                vmax - center; // after placing box at origin, vmax and vmin are symetric.

            double xscale, yscale, zscale;
            xscale = fabs(scale_point[0]);
            yscale = fabs(scale_point[1]);
            zscale = fabs(scale_point[2]);
            double scale = std::max(std::max(xscale, yscale), zscale);
            if (scale > 300) {
                tbb::parallel_for(
                    tbb::blocked_range<int>(0, V.size()),
                    [&](tbb::blocked_range<int> r) {
                        for (int i = r.begin(); i < r.end(); i++) {
                            V[i] = V[i] / scale;
                        }
                    });
            }

            constexpr int multi = 1000;
            tbb::parallel_for(tbb::blocked_range<int>(0, V.size()), [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    list_v[i].morton = Resorting::MortonCode64(
                        int(V[i][0] * multi),
                        int(V[i][1] * multi),
                        int(V[i][2] * multi));
                    list_v[i].order = i;
                }
            });

            const auto morton_compare = [](const sortstruct& a, const sortstruct& b) {
                return (a.morton < b.morton);
            };

            tbb::parallel_sort(list_v.begin(), list_v.end(), morton_compare);

            int interval = list_v.size() / NUM_THREADS + 1;

            tbb::parallel_for(
                tbb::blocked_range<int>(0, list_v.size()),
                [&](tbb::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        m_vertex_attribute[list_v[i].order].partition_id = i / interval;
                    }
                });
        });
    }

    size_t get_partition_id(const Tuple& loc) const
    {
        return m_vertex_attribute[loc.vid(*this)].partition_id;
    }


    double get_length2(const Tuple& l) const;

    ////// Attributes related

    void write_msh(std::string file);
    void output_faces(std::string file, std::function<bool(const FaceAttributes&)> cond);

    void init_from_delaunay_box_mesh(const std::vector<Eigen::Vector3d>& vertices);

    void finalize_triangle_insertion(const std::vector<std::array<size_t, 3>>& faces);

    void init_from_input_surface(
        const std::vector<Vector3d>& vertices,
        const std::vector<std::array<size_t, 3>>& faces,
        const std::vector<size_t>& partition_id);
    bool triangle_insertion_before(const std::vector<Tuple>& faces) override;
    bool triangle_insertion_after(const std::vector<std::vector<Tuple>>& new_faces) override;


public:
    void split_all_edges();
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& loc) override;

    void smooth_all_vertices();
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    /**
     * A short-cut to perform laplacian smoothing on the input surface
     */
    void smooth_input(const int n_iterations);

    void collapse_all_edges(bool is_limit_length = true);
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;

    void swap_all_edges_44();
    bool swap_edge_44_before(const Tuple& t) override;
    bool swap_edge_44_after(const Tuple& t) override;

    void swap_all_edges();
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    void swap_all_faces();
    bool swap_face_before(const Tuple& t) override;
    bool swap_face_after(const Tuple& t) override;

    /**
     * @brief Inversion check using only floating point numbers.
     */
    bool is_inverted_f(const Tuple& loc) const;
    bool is_inverted(const Tuple& loc) const;
    double get_quality(const Tuple& loc) const;

    /**
     * @brief Round a vertex position to floating point.
     *
     * Only rounds the vertex position, if it does not cause inverted elements.
     *
     * @return True if successful or already rounded, false otherwise.
     */
    bool round(const Tuple& loc);
    //
    bool is_edge_on_surface(const Tuple& loc);
    bool is_edge_on_bbox(const Tuple& loc);
    //
    bool adjust_sizing_field(double max_energy);
    void mesh_improvement(int max_its = 80);
    std::tuple<double, double> local_operations(
        const std::array<int, 4>& ops,
        bool collapse_limit_length = true);
    std::tuple<double, double> get_max_avg_energy();

    bool check_attributes();

    std::vector<std::array<size_t, 3>> get_faces_by_condition(
        std::function<bool(const FaceAttributes&)> cond);

    bool invariants(const std::vector<Tuple>& t) override; // this is now automatically checked

    // debug use
    std::atomic<int> cnt_split = 0, cnt_collapse = 0, cnt_swap = 0;

private:
    // tags: correspondence map from new tet-face node indices to in-triangle ids.
    // built up while triangles are inserted.
    tbb::concurrent_map<std::array<size_t, 3>, std::vector<int>> tet_face_tags;

    struct TriangleInsertionLocalInfoCache
    {
        // local info: for each face insertion
        int face_id;
        std::vector<std::array<size_t, 3>> old_face_vids;
    };
    tbb::enumerable_thread_specific<TriangleInsertionLocalInfoCache> triangle_insertion_local_cache;

    ////// Operations

    struct SplitInfoCache
    {
        //        VertexAttributes vertex_info;
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
    tbb::enumerable_thread_specific<SplitInfoCache> split_cache;

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

        std::vector<std::array<size_t, 2>> failed_edges;

        std::map<std::pair<size_t, size_t>, int> edge_link;
        std::map<size_t, int> vertex_link;
        size_t global_nonmani_ver_cnt;

        // debug use
        std::vector<size_t> one_ring_surface_vertices;
        std::vector<std::pair<size_t, size_t>> one_ring_surface_edges;
        std::vector<std::array<size_t, 3>> one_ring_surface;

        // for geometry preservation
        std::vector<size_t> edge_incident_param_type;
    };
    tbb::enumerable_thread_specific<CollapseInfoCache> collapse_cache;


    struct SwapInfoCache
    {
        double max_energy;
        std::map<std::array<size_t, 3>, FaceAttributes> changed_faces;
        int64_t tet_tag;
    };
    tbb::enumerable_thread_specific<SwapInfoCache> swap_cache;

public:
    void insertion_by_volumeremesher(
        const std::vector<Vector3d>& vertices,
        const std::vector<std::array<size_t, 3>>& faces,
        std::vector<Vector3r>& v_rational,
        std::vector<std::array<size_t, 3>>& facets_after,
        std::vector<bool>& is_v_on_input,
        std::vector<std::array<size_t, 4>>& tets_after,
        std::vector<bool>& tet_face_on_input_surface);

    /**
     * @brief Init mesh from arrays generated by the volume remesher.
     *
     * @param v_rational Vertex positions in rational number type.
     * @param is_v_on_input A vector of bools identifying if a vertex is on the input.
     * @param tets Vector of tet vertex IDs, e.g., [[v0,v1,v2,v3],[...],...].
     * @param tet_face_on_input_surface A vector of bools identifying if the face is on the input.
     * The vector contains 4 faces for each tet. If a face is on the input, all incident tets must
     * mark that face as input.
     *
     */
    void init_from_Volumeremesher(
        const std::vector<Vector3r>& v_rational,
        const std::vector<bool>& is_v_on_input,
        const std::vector<std::array<size_t, 4>>& tets,
        const std::vector<bool>& tet_face_on_input_surface);

    /**
     * @brief Init from meshes image.
     *
     * @param V #Vx3 vertices of the tet mesh
     * @param T #Tx4 vertex IDs for all tets
     * @param F #Fx3 vertex IDs of all embedded faces
     * @param T_tags #Tx1 image data represented by the individual tets
     */
    void init_from_image(const MatrixXr& V, const MatrixXi& T, const VectorXi& T_tags);
    void init_from_image(const MatrixXd& V, const MatrixXi& T, const VectorXi& T_tags);

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
    void find_open_boundary();
    /**
     * @brief Checks if an edge COULD be an open boundary edge.
     *
     * The method performs two checks. First, it checks if the two vertices are marked as on the
     * open boundary. Second, it checks if the edge is within the open boundary envelope.
     * Note that these checks are not sufficient to guarantee that an edge is actually on the open
     * boundary! For example, an almost degenerate triangle with two edges on the open boundary
     * could cause a false positive result.
     */
    bool is_open_boundary_edge(const Tuple& e);
    bool is_open_boundary_edge(const std::array<size_t, 2>& e);

    // for topology preservation
    int count_vertex_links(const Tuple& v);
    int count_edge_links(const Tuple& e);

    // for boolean operations
    int flood_fill();

    void write_vtu(const std::string& path);

    void write_surface(const std::string& path) const;

    // initialize sizing field (for topology preservation)
    void init_sizing_field();
};

} // namespace wmtk::components::image_simulation
