#pragma once

#include "Parameters.h"
#include "common.h"

#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/DisableWarnings.hpp>
#include <wmtk/utils/EnableWarnings.hpp>

#include <wmtk/TetMesh.h>

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
    }

    // Stores the attributes attached to simplices
    std::vector<VertexAttributes> m_vertex_attribute;
    std::vector<EdgeAttributes> m_edge_attribute;
    std::vector<FaceAttributes> m_face_attribute;
    std::vector<TetAttributes> m_tet_attribute;

    void resize_attributes(size_t v, size_t e, size_t t, size_t tt) override
    {
        m_vertex_attribute.resize(v);
        m_edge_attribute.resize(e);
        m_face_attribute.resize(t);
        m_tet_attribute.resize(tt);
    }

    void smoothing(const Tuple& t);

    void output_mesh(std::string file) const;

    class InputSurface
    {
    public:
        std::vector<Vector3d> vertices;
        std::vector<std::array<size_t, 3>> faces;
        // can add other input tags;

        Parameters params;

        InputSurface(
            const std::vector<Vector3d>& _vertices,
            const std::vector<std::array<size_t, 3>>& _faces)
            : vertices(_vertices)
            , faces(_faces)
        {
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

        bool remove_duplicates(
            std::vector<Vector3d>& out_vertices,
            std::vector<std::array<size_t, 3>>& out_faces) const;
    };

    struct TriangleInsertionInfoCache
    {
        std::vector<std::array<int, 4>> surface_f_ids;
        size_t face_id;
    } triangle_insertion_cache; // todo: change for parallel

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
    void triangle_insertion(const InputSurface& input_surface);

    void insertion_update_surface_tag(
        size_t t_id,
        size_t new_t_id,
        int config_id,
        int diag_config_id,
        int index) override;

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

    void consolidate_mesh();
    //    void consolidate_mesh_attributes();
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
class cmp_l
{
private:
    const TetWild& m_tw;

public:
    cmp_l(const TetWild& tw)
        : m_tw(tw)
    {}

    bool operator()(const ElementInQueue& e1, const ElementInQueue& e2)
    {
        if (e1.weight == e2.weight) return e1.edge.vid(m_tw) > e2.edge.vid(m_tw);
        return e1.weight < e2.weight;
    }
};
class cmp_s
{
private:
    const TetWild& m_tw;

public:
    cmp_s(const TetWild& tw)
        : m_tw(tw)
    {}
    bool operator()(const ElementInQueue& e1, const ElementInQueue& e2)
    {
        if (e1.weight == e2.weight) return e1.edge.vid(m_tw) < e2.edge.vid(m_tw);
        return e1.weight > e2.weight;
    }
};

} // namespace tetwild