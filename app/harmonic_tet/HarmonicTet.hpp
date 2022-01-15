#pragma once

#include <wmtk/TetMesh.h>

#include <Eigen/Core>
#include <memory>

namespace harmonic_tet {

class HarmonicTet : public wmtk::TetMesh
{
public:
    using VertexAttributes = Eigen::Vector3d;
    struct TetAttributes
    {
    };

    HarmonicTet(
        const std::vector<VertexAttributes>& _vertex_attribute,
        const std::vector<std::array<size_t, 4>>& tets)
    {
        m_vertex_attribute = _vertex_attribute;
        m_tet_attribute = std::vector<TetAttributes>(tets.size());
        init(m_vertex_attribute.size(), tets);
    }
    HarmonicTet(){};
    ~HarmonicTet(){};

    ////// Attributes related
    // Stores the attributes attached to simplices
    std::vector<VertexAttributes> m_vertex_attribute;
    std::vector<TetAttributes> m_tet_attribute;

    void resize_vertex_attributes(size_t v) override { m_vertex_attribute.resize(v); }
    void resize_tet_attributes(size_t t) override { m_tet_attribute.resize(t); }

    void move_tet_attribute(size_t from, size_t to) override
    {
        m_tet_attribute[to] = std::move(m_tet_attribute[from]);
    }
    void move_vertex_attribute(size_t from, size_t to) override
    {
        m_vertex_attribute[to] = std::move(m_vertex_attribute[from]);
    }

    void output_mesh(std::string file) const;

    ////// Operations

    struct SwapInfoCache
    {
        double max_energy = 0.;
    } edgeswap_cache, faceswap_cache; // todo: change for parallel

    void smooth_all_vertices();
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    void swap_all_edges();
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    void swap_all_faces();
    bool swap_face_before(const Tuple& t) override;
    bool swap_face_after(const Tuple& t) override;

    bool is_inverted(const Tuple& loc);
    double get_quality(const Tuple& loc);
};

} // namespace harmonic_tet
