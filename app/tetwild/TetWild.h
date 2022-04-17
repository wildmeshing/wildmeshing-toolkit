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
#include <tbb/concurrent_priority_queue.h>
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
    Vector3r m_pos;
    Vector3d m_posf;
    bool m_is_rounded = false;

    bool m_is_on_surface = false;
    std::vector<int> on_bbox_faces;
    bool m_is_outside = false;

    Scalar m_sizing_scalar = 1;
    Scalar m_scalar = 1;
    bool m_is_freezed = false;

    size_t partition_id = 0;

    VertexAttributes(){};
    VertexAttributes(const Vector3r& p);
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
    {
        NUM_THREADS = _num_threads;
        p_vertex_attrs = &m_vertex_attribute;
        p_face_attrs = &m_face_attribute;
        p_tet_attrs = &m_tet_attribute;
        m_collapse_check_link_condition = false;
    }

    ~TetWild() {}
    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    using FaceAttCol = wmtk::AttributeCollection<FaceAttributes>;
    using TetAttCol = wmtk::AttributeCollection<TetAttributes>;
    VertAttCol m_vertex_attribute;
    FaceAttCol m_face_attribute;
    TetAttCol m_tet_attribute;

    void create_mesh_attributes(
        const std::vector<VertexAttributes>& _vertex_attribute,
        const std::vector<TetAttributes>& _tet_attribute)
    {
        auto n_tet = _tet_attribute.size();
        m_vertex_attribute.resize(_vertex_attribute.size());
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

    void output_mesh(std::string file);
    void output_faces(std::string file, std::function<bool(const FaceAttributes&)> cond);

    void init_from_delaunay_box_mesh(const std::vector<Eigen::Vector3d>& vertices);

    void finalize_triangle_insertion(
        const std::vector<std::array<size_t, 3>>& faces);

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
    void filter_outside(
        const std::vector<Vector3d>& vertices,
        const std::vector<std::array<size_t, 3>>& faces,
        bool remove_ouside = true);

    bool check_attributes();

    std::vector<std::array<size_t, 3>> get_faces_by_condition(
        std::function<bool(const FaceAttributes&)> cond);

    bool invariants(const std::vector<Tuple>& t) override; // this is now automatically checked

    double get_length2(const Tuple& loc) const;
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
};

} // namespace tetwild
