#pragma once

#include <wmtk/TriMesh.h>
#include <filesystem>
#include <functional>
#include <map>
#include <wmtk/Types.hpp>

namespace wmtk::io {

class TriVTUWriter
{
private:
    using Tuple = typename TriMesh::Tuple;

public:
    /**
     * @brief Construct a VTU writer for the given mesh.
     *
     * The IDs in the written mesh might not be the same as in inside the code. If you want these
     * two to match, you must consolidate before writing. Additionally, edge IDs will never match.
     *
     * If you want to know the IDs in your code, you can add an attribute like:
     * ```
     * TriMesh m;
     * writer.add_edge_attribute("eid", [&m](int i) { return VectorXd::Constant(1, i); });
     * ```
     */
    TriVTUWriter(const TriMesh& mesh);

    /**
     * @brief Add vertex positions.
     *
     * If no positions are provided, all vertices will be placed at (0,0,0).
     *
     * @param f A function that takes the vertex ID as input and returns the position.
     */
    void add_vertex_positions(const std::function<VectorXd(const size_t)>& f);

    /**
     * @brief Add vertex attribute.
     *
     * Examples:
     * ```
     * TriMesh m;
     * writer.add_vertex_attribute("abc", [&m](int i) { return m.vertex_attrs[i].pos; });
     * writer.add_vertex_attribute("vid", [&m](int i) { return VectorXd::Constant(1, i); });
     * ```
     *
     * @param f A function that takes the vertex ID as input and returns the attribute value for
     * that vertex.
     */
    void add_vertex_attribute(
        const std::string& name,
        const std::function<VectorXd(const size_t)>& f);
    void add_vertex_attribute(
        const std::string& name,
        const std::function<double(const size_t)>& f);
    /**
     * @brief Add edge attribute.
     *
     * See `add_vertex_attribute` for details.
     */
    void add_edge_attribute(
        const std::string& name,
        const std::function<VectorXd(const size_t)>& f);
    void add_edge_attribute(const std::string& name, const std::function<double(const size_t)>& f);

    /**
     * @brief Add triangle attribute.
     *
     * See `add_vertex_attribute` for details.
     */
    void add_triangle_attribute(
        const std::string& name,
        const std::function<VectorXd(const size_t)>& f);
    void add_triangle_attribute(
        const std::string& name,
        const std::function<double(const size_t)>& f);

    /**
     * @brief Write the triangle mesh with triangle and vertex attributes.
     *
     * @param filename The file the mesh is written to. Should have the extension .VTU.
     */
    bool write_triangles(const std::filesystem::path& filename);

    /**
     * @brief Write the edge mesh with edge and vertex attributes.
     *
     * @param filename The file the mesh is written to. Should have the extension .VTU.
     */
    bool write_edges(const std::filesystem::path& filename);

private:
    const TriMesh& m_mesh;

    MatrixXd m_V; // vertex positions
    MatrixXi m_E; // edge - vids
    MatrixXi m_F; // face - vids

    std::map<std::string, MatrixXd> m_V_attributes;
    std::map<std::string, MatrixXd> m_E_attributes;
    std::map<std::string, MatrixXd> m_F_attributes;
};

} // namespace wmtk::io
