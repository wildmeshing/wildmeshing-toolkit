#pragma once

#include <wmtk/TetMesh.h>
#include "Parameters.h"
#include "common.h"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <memory>

namespace tetwild {

class VertexAttributes
{
public:
    Vector3 m_pos;
    Vector3d m_posf;

    bool m_is_on_surface;
    bool m_is_on_boundary;
    bool m_is_on_bbox;
    bool m_is_outside;

    Scalar m_sizing_scalars;
    Scalar m_scalars;
    bool m_is_freezed;
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

    int m_is_surface_fs;
    int m_is_bbox_fs;
    int m_opp_t_ids;
    int m_surface_tags;
};

class TetAttributes
{
public:
    Scalar m_qualities;
    Scalar m_scalars;
    bool m_is_outside;
};

class TetWild : public wmtk::TetMesh
{
public:
    Parameters& m_params;
    fastEnvelope::FastEnvelope& m_envelope;

    TetWild(Parameters& _m_params, fastEnvelope::FastEnvelope& _m_envelope)
        : m_params(_m_params)
        , m_envelope(_m_envelope)
    {}

    ~TetWild() {}

    void create_mesh_attributes(
        const std::vector<VertexAttributes>& _vertex_attribute,
        const std::vector<TetAttributes>& _tet_attribute)
    {
        m_vertex_attribute = _vertex_attribute;
        m_tet_attribute = _tet_attribute;
        auto n_tet = m_tet_attribute.size();
        resize_edge_attributes(6 * n_tet);
        resize_face_attributes(4 * n_tet);
    }

    ////// Attributes related
    // Stores the attributes attached to simplices
    std::vector<VertexAttributes> m_vertex_attribute;
    std::vector<EdgeAttributes> m_edge_attribute;
    std::vector<FaceAttributes> m_face_attribute;
    std::vector<TetAttributes> m_tet_attribute;

    void resize_vertex_attributes(size_t v) override { m_vertex_attribute.resize(v); }
    void resize_edge_attributes(size_t e) override { m_edge_attribute.resize(e); }
    void resize_face_attributes(size_t f) override { m_face_attribute.resize(f); }
    void resize_tet_attributes(size_t t) override { m_tet_attribute.resize(t); }


    void move_face_attribute(size_t from, size_t to) override
    {
        m_face_attribute[to] = std::move(m_face_attribute[from]);
    }
    void move_edge_attribute(size_t from, size_t to) override
    {
        m_edge_attribute[to] = std::move(m_edge_attribute[from]);
    }
    void move_tet_attribute(size_t from, size_t to) override
    {
        m_tet_attribute[to] = std::move(m_tet_attribute[from]);
    }
    void move_vertex_attribute(size_t from, size_t to) override
    {
        m_vertex_attribute[to] = std::move(m_vertex_attribute[from]);
    }

    void output_mesh(std::string file);

    class InputSurface
    {
    public:
        std::vector<Vector3d> vertices;
        std::vector<std::array<size_t, 3>> faces;
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

    struct TriangleInsertionInfoCache
    {
        // global info: throughout the whole insertion
        InputSurface input_surface;
        std::vector<std::array<int, 4>> surface_f_ids;
        std::map<std::array<size_t, 3>, std::vector<int>> tet_face_tags;
        std::vector<bool> is_matched;

        // local info: for each face insertion
        std::vector<bool> is_visited;
        int face_id;
        std::vector<std::array<size_t, 3>> old_face_vids;
    };
    TriangleInsertionInfoCache triangle_insertion_cache;

    ////// Operations

    struct SplitInfoCache
    {
        VertexAttributes vertex_info;
    } split_cache; // todo: change for parallel

    struct CollapseInfoCache
    {
        double max_energy;
        double edge_length;
    } collapse_cache; // todo: change for parallel


    struct SwapInfoCache
    {
        double max_energy;
    } edgeswap_cache, faceswap_cache; // todo: change for parallel


    void construct_background_mesh(const InputSurface& input_surface);
    void match_insertion_faces(const InputSurface& input_surface, std::vector<bool>& is_matched);
    //
//    void add_tet_centroid(const std::array<size_t, 4>& vids) override;
    void add_tet_centroid(const Tuple& t) override;
    //
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

    void collapse_all_edges();
    bool collapse_before(const Tuple& t) override;
    bool collapse_after(const Tuple& t) override;

    void swap_all_edges();
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    void swap_all_faces();
    bool swap_face_before(const Tuple& t) override;
    bool swap_face_after(const Tuple& t) override;

    bool is_inverted(const Tuple& loc);
    double get_quality(const Tuple& loc);

    bool vertex_invariant(const Tuple& t) override;
    bool tetrahedron_invariant(const Tuple& t) override;
    std::atomic<int> cnt_split = 0, cnt_collapse = 0, cnt_swap = 0;
};

class ElementInQueue
{
public:
    wmtk::TetMesh::Tuple edge;
    double weight;

    ElementInQueue() {}
    ElementInQueue(const wmtk::TetMesh::Tuple& e, double w)
        : edge(e)
        , weight(w)
    {}
};
struct cmp_l
{
    bool operator()(const ElementInQueue& e1, const ElementInQueue& e2)
    {
        if (e1.weight == e2.weight) return e1.edge < e2.edge;
        return e1.weight < e2.weight;
    }
};
struct cmp_s
{
    bool operator()(const ElementInQueue& e1, const ElementInQueue& e2)
    {
        if (e1.weight == e2.weight) return e1.edge < e2.edge;
        return e1.weight > e2.weight;
    }
};

} // namespace tetwild
