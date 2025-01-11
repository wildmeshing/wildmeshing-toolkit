#include "ParaviewWriter.hpp"


#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>

#include <paraviewo/VTMWriter.hpp>
#include <paraviewo/VTUWriter.hpp>

// #include <paraviewo/HDF5VTUWriter.hpp>

#include <sstream>

namespace wmtk::io {

ParaviewWriter::ParaviewInternalWriter::ParaviewInternalWriter()
{
    m_paraview_file = std::make_shared<paraviewo::VTUWriter>();
    // m_paraview_file = std::make_shared<paraviewo::HDF5VTUWriter>();
}

void ParaviewWriter::ParaviewInternalWriter::init(
    const std::filesystem::path& filename,
    const std::string& vertices_name,
    const Eigen::MatrixXi& elements,
    const bool enabled)
{
    m_vertices_name = vertices_name;
    m_elements = elements;

    m_filename = filename;
    m_enabled = enabled;
}

ParaviewWriter::ParaviewInternalWriter::~ParaviewInternalWriter()
{
    if (m_enabled) m_paraview_file->write_mesh(m_filename.string(), m_vertices, m_elements);
}


void ParaviewWriter::ParaviewInternalWriter::write(
    const std::string& name,
    const int64_t stride,
    const std::vector<double>& val,
    const bool is_cell_field)
{
    Eigen::MatrixXd tmp =
        Eigen::Map<const Eigen::MatrixXd>(&val[0], stride, val.size() / stride).transpose();

    if (stride == 1 || stride == 2 || stride == 3) {
        if (is_cell_field) {
            m_paraview_file->add_cell_field(name, tmp);
        } else {
            m_paraview_file->add_field(name, tmp);
        }
    } else if (stride % 3 == 0) {
        for (int64_t i = 0; i < stride; i += 3) {
            if (is_cell_field) {
                m_paraview_file->add_cell_field(
                    name + "_" + std::to_string(i / 3),
                    tmp.block(0, i, tmp.rows(), 3));
            } else {
                m_paraview_file->add_field(
                    name + "_" + std::to_string(i / 3),
                    tmp.block(0, i, tmp.rows(), 3));
            }
        }
    } else if (stride % 2 == 0) {
        for (int64_t i = 0; i < stride; i += 2) {
            if (is_cell_field) {
                m_paraview_file->add_cell_field(
                    name + "_" + std::to_string(i / 2),
                    tmp.block(0, i, tmp.rows(), 2));
            } else {
                m_paraview_file->add_field(
                    name + "_" + std::to_string(i / 2),
                    tmp.block(0, i, tmp.rows(), 2));
            }
        }
    } else {
        for (int64_t i = 0; i < stride; ++i) {
            if (is_cell_field) {
                m_paraview_file->add_cell_field(name + "_" + std::to_string(i), tmp.col(i));
            } else {
                m_paraview_file->add_field(name + "_" + std::to_string(i), tmp.col(i));
            }
        }
    }
}

ParaviewWriter::ParaviewWriter(
    const std::filesystem::path& filename,
    const std::string& vertices_name,
    const Mesh& mesh,
    bool write_points,
    bool write_edges,
    bool write_faces,
    bool write_tetrahedra)
    : m_vertices_name(vertices_name)
{
    m_enabled[0] = write_points;
    m_enabled[1] = write_edges;
    m_enabled[2] = write_faces;
    m_enabled[3] = write_tetrahedra;

    std::array<Eigen::MatrixXi, 4> cells;

    for (size_t i = 0; i < 4; ++i) {
        const auto pt = PrimitiveType(i);
        if (m_enabled[i]) {
            // include deleted tuples so that attributes are aligned
            const auto tuples = mesh.get_all(pt, true);
            cells[i].resize(tuples.size(), i + 1);

            for (size_t j = 0; j < tuples.size(); ++j) {
                const auto& t = tuples[j];
                if (t.is_null()) {
                    for (size_t d = 0; d < cells[i].cols(); ++d) {
                        cells[i](j, d) = 0;
                    }
                } else {
                    int64_t vid = mesh.id(t, PrimitiveType::Vertex);
                    cells[i](j, 0) = vid;
                    if (i > 0) {
                        auto t1 = mesh.switch_tuple(t, PrimitiveType::Vertex);

                        cells[i](j, 1) = mesh.id(t1, PrimitiveType::Vertex);
                    }
                    if (i > 1) {
                        auto t1 = mesh.switch_tuple(t, PrimitiveType::Edge);
                        auto t2 = mesh.switch_tuple(t1, PrimitiveType::Vertex);

                        cells[i](j, 2) = mesh.id(t2, PrimitiveType::Vertex);
                    }
                    if (i > 2) {
                        auto t1 = mesh.switch_tuple(t, PrimitiveType::Triangle);
                        auto t2 = mesh.switch_tuple(t1, PrimitiveType::Edge);
                        auto t3 = mesh.switch_tuple(t2, PrimitiveType::Vertex);

                        cells[i](j, 3) = mesh.id(t3, PrimitiveType::Vertex);
                    }
                }
            }
        }
    }

    m_writers[0].init(filename.string() + "_verts.vtu", vertices_name, cells[0], m_enabled[0]);
    m_writers[1].init(filename.string() + "_edges.vtu", vertices_name, cells[1], m_enabled[1]);
    m_writers[2].init(filename.string() + "_faces.vtu", vertices_name, cells[2], m_enabled[2]);
    m_writers[3].init(filename.string() + "_tets.vtu", vertices_name, cells[3], m_enabled[3]);

    if (m_enabled[0] + m_enabled[1] + m_enabled[2] + m_enabled[3] > 1) {
        paraviewo::VTMWriter vtm;
        if (m_enabled[0]) vtm.add_dataset("verts", "mesh", filename.string() + "_verts.vtu");
        if (m_enabled[1]) vtm.add_dataset("edges", "mesh", filename.string() + "_edges.vtu");
        if (m_enabled[2]) vtm.add_dataset("faces", "mesh", filename.string() + "_faces.vtu");
        if (m_enabled[3]) vtm.add_dataset("tets", "mesh", filename.string() + "_tets.vtu");

        vtm.save(filename.string() + ".vtm");
    }
}

void ParaviewWriter::write(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<int64_t>& val,
    const int64_t default_val)
{
    std::vector<double> tmp;
    tmp.reserve(val.size());
    for (const auto& v : val) tmp.push_back(v);

    write_internal(name, type, stride, tmp);
}

void ParaviewWriter::write(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<double>& val,
    const double default_val)
{
    write_internal(name, type, stride, val);
}

void ParaviewWriter::write(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<char>& val,
    const char default_val)
{
    std::vector<double> tmp;
    tmp.reserve(val.size());
    for (const auto& v : val) tmp.push_back(v);

    write_internal(name, type, stride, tmp);
}


void ParaviewWriter::write(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<Rational>& val,
    const Rational& default_val)
{
    std::vector<double> tmp;
    tmp.reserve(val.size());
    for (const auto& v : val) tmp.push_back(double(v));

    write_internal(name, type, stride, tmp);
}

void ParaviewWriter::write_internal(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<double>& val)
{
    if (!m_write) return;

    if (name == m_vertices_name) {
        assert(stride == 2 || stride == 3);

        Eigen::MatrixXd V =
            Eigen::Map<const Eigen::MatrixXd>(&val[0], stride, val.size() / stride).transpose();

        for (int i = 0; i < m_writers.size(); ++i) {
            if (m_enabled[i]) m_writers[i].vertices() = V;
        }
    } else if (type == 0) { // vertex attrs are always written
        for (size_t i = 0; i < m_writers.size(); ++i) {
            if (m_enabled[i]) m_writers[i].write(name, stride, val, false);
        }
    } else if (m_enabled[type]) {
        m_writers[type].write(name, stride, val, true);
    }
}


} // namespace wmtk::io
