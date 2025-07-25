#include "TriVTUWriter.hpp"

#include <paraviewo/VTUWriter.hpp>

namespace wmtk::io {

TriVTUWriter::TriVTUWriter(const TriMesh& mesh)
    : m_mesh(mesh)
{
    const std::vector<Tuple> face_tuples = m_mesh.get_faces();
    const std::vector<Tuple> v_tuples = m_mesh.get_vertices();

    std::map<size_t, size_t> vid_map; // map from vid to consecutive
    for (int i = 0; i < v_tuples.size(); ++i) {
        const size_t vid = v_tuples[i].vid(m_mesh);
        vid_map[vid] = i;
    }

    if (face_tuples.empty()) {
        log_and_throw_error("Cannot print mesh without faces.");
    }

    // F
    m_F.resize(face_tuples.size(), 3);
    for (int i = 0; i < face_tuples.size(); ++i) {
        const auto v = m_mesh.oriented_tri_vertices(face_tuples[i]);
        for (int j = 0; j < v.size(); ++j) {
            m_F(i, j) = vid_map[v[j].vid(m_mesh)];
        }
    }

    // E
    const std::vector<Tuple> edge_tuples = m_mesh.get_edges();
    m_E.resize(edge_tuples.size(), 2);
    for (int i = 0; i < edge_tuples.size(); ++i) {
        const Tuple& t = edge_tuples[i];
        const size_t eid = t.eid(m_mesh);
        const size_t v0 = vid_map[t.vid(m_mesh)];
        const size_t v1 = vid_map[t.switch_vertex(m_mesh).vid(m_mesh)];
        m_E.row(i) = Vector2i(v0, v1);
    }

    // V (dummy)
    m_V = MatrixXd::Zero(m_mesh.vert_capacity(), 3);
}

bool TriVTUWriter::write_triangles(const std::filesystem::path& filename)
{
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

bool TriVTUWriter::write_edges(const std::filesystem::path& filename)
{
    paraviewo::VTUWriter writer;
    for (const auto& [name, attr] : m_V_attributes) {
        writer.add_field(name, attr);
    }
    for (const auto& [name, attr] : m_E_attributes) {
        writer.add_cell_field(name, attr);
    }

    bool r = writer.write_mesh(filename.string(), m_V, m_E);
    return r;
}

void TriVTUWriter::add_vertex_positions(const std::function<VectorXd(const int)>& f)
{
    const std::vector<Tuple> tuples = m_mesh.get_vertices();

    m_V.resize(tuples.size(), f(tuples[0].vid(m_mesh)).size());

    for (int i = 0; i < tuples.size(); ++i) {
        const Tuple& t = tuples[i];
        const size_t id = t.vid(m_mesh);
        m_V.row(i) = f(id);
    }
}

void TriVTUWriter::add_vertex_attribute(
    const std::string& name,
    const std::function<VectorXd(const int)>& f)
{
    const std::vector<Tuple> tuples = m_mesh.get_vertices();

    MatrixXd attr;
    attr.resize(tuples.size(), f(tuples[0].vid(m_mesh)).size());

    for (int i = 0; i < tuples.size(); ++i) {
        const Tuple& t = tuples[i];
        const size_t id = t.vid(m_mesh);
        attr.row(i) = f(id);
    }

    m_V_attributes[name] = attr;
}

void TriVTUWriter::add_vertex_attribute(
    const std::string& name,
    const std::function<double(const int)>& f)
{
    add_vertex_attribute(name, [&f](int i) { return VectorXd::Constant(1, f(i)); });
}

void TriVTUWriter::add_edge_attribute(
    const std::string& name,
    const std::function<VectorXd(const int)>& f)
{
    const std::vector<Tuple> tuples = m_mesh.get_edges();

    MatrixXd attr;
    attr.resize(tuples.size(), f(tuples[0].eid(m_mesh)).size());

    for (int i = 0; i < tuples.size(); ++i) {
        const Tuple& t = tuples[i];
        const size_t id = t.eid(m_mesh);
        attr.row(i) = f(id);
    }

    m_E_attributes[name] = attr;
}

void TriVTUWriter::add_edge_attribute(
    const std::string& name,
    const std::function<double(const int)>& f)
{
    add_edge_attribute(name, [&f](int i) { return VectorXd::Constant(1, f(i)); });
}

void TriVTUWriter::add_triangle_attribute(
    const std::string& name,
    const std::function<VectorXd(const int)>& f)
{
    const std::vector<Tuple> tuples = m_mesh.get_faces();

    MatrixXd attr;
    attr.resize(tuples.size(), f(tuples[0].vid(m_mesh)).size());

    for (int i = 0; i < tuples.size(); ++i) {
        const Tuple& t = tuples[i];
        const size_t id = t.fid(m_mesh);
        attr.row(i) = f(id);
    }

    m_F_attributes[name] = attr;
}

void TriVTUWriter::add_triangle_attribute(
    const std::string& name,
    const std::function<double(const int)>& f)
{
    add_triangle_attribute(name, [&f](int i) { return VectorXd::Constant(1, f(i)); });
}

} // namespace wmtk::io
