#pragma once

#include <tbb/concurrent_map.h>
#include <wmtk/ConcurrentTetMesh.h>
#include <wmtk/utils/PartitionMesh.h>
#include "Parameters.h"
#include "common.h"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/concurrent_map.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/enumerable_thread_specific.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <wmtk/utils/PartitionMesh.h>
#include <memory>

namespace tetwild {

class VertexAttributes
{
public:
    Vector3 m_pos;
    Vector3d m_posf;
    bool m_is_rounded = false;

    bool m_is_on_surface = false;
    std::vector<int> on_bbox_faces;
    bool m_is_outside = false;

    Scalar m_sizing_scalar = 1;
    Scalar m_scalar = 1;
    bool m_is_freezed = false;

    size_t partition_id = 0;
};

class EdgeAttributes
{
public:
    // Scalar length;
};

class FaceAttributes
{
public:
    Scalar tag;

    bool m_is_surface_fs = false; // 0; 1
    int m_is_bbox_fs = -1; //-1; 0~5

    int m_surface_tags = -1;

    void reset()
    {
        m_is_surface_fs = false;
        m_is_bbox_fs = -1;
        m_surface_tags = -1;
    }

    void merge(const FaceAttributes& attr)
    {
        m_is_surface_fs = m_is_surface_fs || attr.m_is_surface_fs;
        if (attr.m_is_bbox_fs >= 0) m_is_bbox_fs = attr.m_is_bbox_fs;
        m_surface_tags = std::max(m_surface_tags, attr.m_surface_tags);
    }
};

class TetAttributes
{
public:
    Scalar m_quality;
    Scalar m_scalar;
    bool m_is_outside;
};

class TetWild : public wmtk::ConcurrentTetMesh
{
public:
    const double MAX_ENERGY = 1e50;

    Parameters& m_params;
    fastEnvelope::FastEnvelope& m_envelope;

    TetWild(Parameters& _m_params, fastEnvelope::FastEnvelope& _m_envelope, int _num_threads = 1)
        : m_params(_m_params)
        , m_envelope(_m_envelope)
        , NUM_THREADS(_num_threads)
    {
        p_vertex_attrs = &m_vertex_attribute;
        p_edge_attrs = &m_edge_attribute;
        p_face_attrs = &m_face_attribute;
        p_tet_attrs = &m_tet_attribute;
        m_collapse_check_link_condition = false;
    }

    ~TetWild() {}
    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes>;
    using FaceAttCol = wmtk::AttributeCollection<FaceAttributes>;
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    VertAttCol m_vertex_attribute;
    EdgeAttCol m_edge_attribute;
    FaceAttCol m_face_attribute;
    TetAttCol m_tet_attribute;

    void create_mesh_attributes(
        const std::vector<VertexAttributes>& _vertex_attribute,
        const std::vector<TetAttributes>& _tet_attribute)
    {
        auto n_tet = _tet_attribute.size();
        m_vertex_attribute.resize(_vertex_attribute.size());
        m_edge_attribute.resize(6 * n_tet);
        m_face_attribute.resize(4 * n_tet);
        m_tet_attribute.resize(n_tet);

        for (auto i = 0; i < _vertex_attribute.size(); i++)
            m_vertex_attribute[i] = _vertex_attribute[i];
        m_tet_attribute.m_attributes = tbb::concurrent_vector<TetAttributes>(_tet_attribute.size());
        for (auto i = 0; i < _tet_attribute.size(); i++) m_tet_attribute[i] = _tet_attribute[i];
    }

    void compute_vertex_partition()
    {
        auto partition_id = partition_TetMesh(*this, NUM_THREADS);
        for (auto i = 0; i < m_vertex_attribute.size(); i++)
            m_vertex_attribute[i].partition_id = partition_id[i];
    }
    size_t get_partition_id(const Tuple& loc) const
    {
        return m_vertex_attribute[loc.vid(*this)].partition_id;
    }

    ////// Attributes related
    int NUM_THREADS = 1;


    void output_mesh(std::string file);
    void output_faces(std::string file, std::function<bool(const FaceAttributes&)> cond);

    class InputSurface
    {
    public:
        std::vector<Vector3d> vertices;
        std::vector<std::array<size_t, 3>> faces;
        std::vector<int> partition_id;
        // can add other input tags;

        Parameters params;

        InputSurface() {}

        void init(
            const std::vector<Vector3d>& _vertices,
            const std::vector<std::array<size_t, 3>>& _faces)
        {
            vertices = _vertices;
            faces = _faces;
            Vector3d min, max;
            for (size_t i = 0; i < vertices.size(); i++) {
                if (i == 0) {
                    min = vertices[i];
                    max = vertices[i];
                    continue;
                }
                for (int j = 0; j < 3; j++) {
                    if (vertices[i][j] < min[j]) min[j] = vertices[i][j];
                    if (vertices[i][j] > max[j]) max[j] = vertices[i][j];
                }
            }

            params.init(min, max);
        }

        bool remove_duplicates(); // inplace func
    };

    struct TriangleInsertionInfoGlobalCache
    {
        // global info: throughout the whole insertion
        InputSurface input_surface;
        tbb::concurrent_map<std::array<size_t, 3>, std::vector<int>> tet_face_tags;
        tbb::concurrent_vector<bool> is_matched;
    };
    TriangleInsertionInfoGlobalCache triangle_insertion_global_cache;

    struct TriangleInsertionLocalInfoCache
    {
        // local info: for each face insertion
        std::vector<bool> is_visited;
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

        std::vector<std::pair<FaceAttributes, std::array<size_t, 3>>> changed_faces;
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
        std::vector<std::array<size_t, 3>> surface_faces;
        std::vector<size_t> changed_tids;

        std::vector<std::array<size_t, 2>> failed_edges;
    };
    tbb::enumerable_thread_specific<CollapseInfoCache> collapse_cache;


    struct SwapInfoCache
    {
        double max_energy;
        std::map<std::array<size_t, 3>, FaceAttributes> changed_faces;
    };
    tbb::enumerable_thread_specific<SwapInfoCache> swap_cache;


    void construct_background_mesh(const InputSurface& input_surface);
    void match_insertion_faces(
        const InputSurface& input_surface,
        tbb::concurrent_vector<bool>& is_matched);
    void setup_attributes();
    //
    void add_tet_centroid(const Tuple& t, size_t vid) override;
    //
    void triangle_insertion_stuff(
        std::vector<tbb::concurrent_queue<std::tuple<size_t, int>>>& insertion_queues,
        tbb::concurrent_queue<size_t>& expired_queue,
        int task_id);
    void triangle_insertion(const InputSurface& input_surface);
    void triangle_insertion_before(const std::vector<Tuple>& faces) override;
    void triangle_insertion_after(
        const std::vector<Tuple>& faces,
        const std::vector<std::vector<Tuple>>& new_faces) override;


    void split_all_edges();
    bool split_before(const Tuple& t) override;
    bool split_after(const Tuple& loc) override;

    void smooth_all_vertices();
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    void collapse_all_edges(bool is_limit_length = true);
    bool collapse_before(const Tuple& t) override;
    bool collapse_after(const Tuple& t) override;


    void swap_all_edges_44();
    bool swap_edge_44_before(const Tuple& t) override;
    bool swap_edge_44_after(const Tuple& t) override;

    void swap_all_edges();
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    void swap_all_faces();
    bool swap_face_before(const Tuple& t) override;
    bool swap_face_after(const Tuple& t) override;

    bool is_inverted(const Tuple& loc) const;
    double get_quality(const Tuple& loc) const;
    bool round(const Tuple& loc);
    //
    bool is_edge_on_surface(const Tuple& loc);
    bool is_edge_on_bbox(const Tuple& loc);
    //
    bool adjust_sizing_field(double max_energy);
    void mesh_improvement(int max_its = 80);
    std::tuple<double, double> local_operations(
        const std::array<int, 4>& ops,
        bool collapse_limite_length = true);
    std::tuple<double, double> get_max_avg_energy();
    void filter_outside(bool remove_ouside = true);

    void check_attributes();

    std::vector<std::array<size_t, 3>> get_faces_by_condition(
        std::function<bool(const FaceAttributes&)> cond);

    bool invariants(const std::vector<Tuple>& t)
        override; // this is now automatically checked

    double get_length2(const Tuple& loc) const;
    // debug use
    std::atomic<int> cnt_split = 0, cnt_collapse = 0, cnt_swap = 0;
};

} // namespace tetwild
