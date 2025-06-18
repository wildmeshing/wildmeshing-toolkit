#pragma once

#include <filesystem>
#include <functional>
#include <map>
#include <paraviewo/VTUWriter.hpp>
#include <wmtk/Types.hpp>

namespace wmtk::io {

template <typename MeshT>
class VTUWriter
{
    using Tuple = typename MeshT::Tuple;

public:
    VTUWriter(MeshT& mesh);

    void add_vertex_positions(const std::function<VectorXd(const int)>& f);

    void add_vertex_attribute(const std::string& name, const std::function<VectorXd(const int)>& f);

    void add_triangle_attribute(
        const std::string& name,
        const std::function<VectorXd(const int)>& f);

    bool write_triangles(const std::filesystem::path& filename);

private:
    MeshT& m_mesh;

    MatrixXd m_V; // vertex positions
    MatrixXi m_E; // edge - vids
    MatrixXi m_F; // face - vids
    MatrixXi m_T; //  tet - vids

    std::map<std::string, MatrixXd> m_V_attributes;
    std::map<std::string, MatrixXd> m_F_attributes;
};

template <typename MeshT>
inline VTUWriter<MeshT>::VTUWriter(MeshT& mesh)
    : m_mesh(mesh)
{
    const std::vector<Tuple> face_tuples = m_mesh.get_faces();

    if (face_tuples.empty()) {
        log_and_throw_error("Cannot print mesh without faces.");
    }

    m_F.resize(face_tuples.size(), 3);
    for (int i = 0; i < face_tuples.size(); ++i) {
        const auto vs = m_mesh.oriented_tri_vids(face_tuples[i]);
        for (int j = 0; j < 3; j++) {
            m_F(i, j) = vs[j];
        }
    }
}

template <typename MeshT>
inline bool VTUWriter<MeshT>::write_triangles(const std::filesystem::path& filename)
{
    assert(m_F.size() > 0);

    if (m_V.size() == 0) {
        log_and_throw_error("Must set vertex positions before writing");
    }

    paraviewo::VTUWriter writer;
    for (const auto& [name, attr] : m_V_attributes) {
        writer.add_field(name, attr);
    }
    for (const auto& [name, attr] : m_F_attributes) {
        writer.add_cell_field(name, attr);
    }

    bool r = writer.write_mesh(filename.string(), m_V, m_F);
    return r;
}

template <typename MeshT>
inline void VTUWriter<MeshT>::add_vertex_positions(const std::function<VectorXd(const int)>& f)
{
    int max_id = m_mesh.vert_capacity();

    if (max_id < 0) {
        log_and_throw_error("Cannot print mesh without vertices.");
    }

    // init V
    m_V.resize(max_id, f(0).size());

    for (int i = 0; i < max_id; ++i) {
        m_V.row(i) = f(i);
    }
}

template <typename MeshT>
inline void VTUWriter<MeshT>::add_vertex_attribute(
    const std::string& name,
    const std::function<VectorXd(const int)>& f)
{
    int max_id = m_mesh.vert_capacity();

    if (max_id < 0) {
        logger().warn("Cannot add attribute {}.", name);
        return;
    }

    MatrixXd attr;
    attr.resize(max_id, f(0).size());

    for (int i = 0; i < max_id; ++i) {
        attr.row(i) = f(i);
    }

    m_V_attributes[name] = attr;
}

template <typename MeshT>
inline void VTUWriter<MeshT>::add_triangle_attribute(
    const std::string& name,
    const std::function<VectorXd(const int)>& f)
{
    int max_id = m_mesh.tri_capacity();

    if (max_id < 0) {
        logger().warn("Cannot add attribute {}.", name);
        return;
    }

    MatrixXd attr;
    attr.resize(max_id, f(0).size());

    for (int i = 0; i < max_id; ++i) {
        attr.row(i) = f(i);
    }

    m_F_attributes[name] = attr;
}

} // namespace wmtk::io
