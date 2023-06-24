#include "ParaviewWriter.hpp"


#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>

#include <paraviewo/HDF5VTUWriter.hpp>

#include <sstream>

namespace wmtk {

ParaviewWriter::ParaviewInternalWriter::ParaviewInternalWriter()
{
    m_paraview_file = std::make_shared<paraviewo::HDF5VTUWriter>();
}

void ParaviewWriter::ParaviewInternalWriter::init(
    const std::filesystem::path& filename,
    const std::string& vertices_name,
    const std::string& elements_name,
    const bool enabled)
{
    m_vertices_name = vertices_name;
    m_elements_name = elements_name;
    m_filename = filename;
    m_enabled = enabled;
}

ParaviewWriter::ParaviewInternalWriter::~ParaviewInternalWriter()
{
    if (m_enabled) m_paraview_file->write_mesh(m_filename, m_vertices, m_elements);
}


void ParaviewWriter::ParaviewInternalWriter::write(
    const std::string& name,
    const long stride,
    const std::vector<double>& val)
{
    if (name == m_elements_name) {
        assert(stride == 2 || stride == 3);

        m_elements = Eigen::Map<const Eigen::MatrixXd>(&val[0], stride, val.size() / stride)
                         .cast<int>()
                         .transpose();
    } else {
        if (stride > 3) {
            logger().warn("Skpping {}, stride {} > 3", name, stride);
            return;
        }

        Eigen::MatrixXd tmp =
            Eigen::Map<const Eigen::MatrixXd>(&val[0], val.size() / stride, stride);
        m_paraview_file->add_cell_field(name, tmp);
    }
}

ParaviewWriter::ParaviewWriter(
    const std::filesystem::path& filename,
    const std::string& vertices_name,
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

    m_writers[0].init(filename.string() + "_verts.hdf5", vertices_name, "m_vf", m_enabled[0]);
    m_writers[1].init(filename.string() + "_edges.hdf5", vertices_name, "m_ef", m_enabled[1]);
    m_writers[2].init(filename.string() + "_faces.hdf5", vertices_name, "m_fv", m_enabled[2]);
    m_writers[3].init(filename.string() + "_tets.hdf5", vertices_name, "m_vf", m_enabled[3]);
}

void ParaviewWriter::write(
    const std::string& name,
    const long type,
    const long stride,
    const std::vector<long>& val)
{
    std::vector<double> tmp;
    tmp.reserve(val.size());
    for (const auto& v : val) tmp.push_back(v);

    write_internal(name, type, stride, tmp);
}

void ParaviewWriter::write(
    const std::string& name,
    const long type,
    const long stride,
    const std::vector<double>& val)
{
    write_internal(name, type, stride, val);
}

void ParaviewWriter::write(
    const std::string& name,
    const long type,
    const long stride,
    const std::vector<char>& val)
{
    std::vector<double> tmp;
    tmp.reserve(val.size());
    for (const auto& v : val) tmp.push_back(v);

    write_internal(name, type, stride, tmp);
}


void ParaviewWriter::write(
    const std::string& name,
    const long type,
    const long stride,
    const std::vector<Rational>& val)
{
    std::vector<double> tmp;
    tmp.reserve(val.size());
    for (const auto& v : val) tmp.push_back(double(v));

    write_internal(name, type, stride, tmp);
}

void ParaviewWriter::write_internal(
    const std::string& name,
    const long type,
    const long stride,
    const std::vector<double>& val)
{
    if (name == m_vertices_name) {
        assert(stride == 2 || stride == 3);

        Eigen::MatrixXd V =
            Eigen::Map<const Eigen::MatrixXd>(&val[0], stride, val.size() / stride).transpose();

        for (int i = 0; i < m_writers.size(); ++i) {
            if (m_enabled[i]) m_writers[i].vertices() = V;
        }

    } else if (m_enabled[type])
        m_writers[type].write(name, stride, val);
}


} // namespace wmtk