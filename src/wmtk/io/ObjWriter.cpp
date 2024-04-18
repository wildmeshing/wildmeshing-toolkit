#include "ObjWriter.hpp"


#include <fstream>
#include <paraviewo/VTUWriter.hpp>
#include <sstream>
#include <wmtk/Mesh.hpp>
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::io {
ObjWriter::ObjWriter() {}

ObjWriter::ObjWriter(
    const std::filesystem::path& filename,
    const std::string& vertices_name,
    const Mesh& mesh)
{
    Eigen::MatrixXi cells;

    // include deleted tuples so that attributes are aligned
    const auto tuples = mesh.get_all(PrimitiveType::Triangle, true);
    cells.resize(tuples.size(), 3);

    for (size_t j = 0; j < tuples.size(); ++j) {
        const auto& t = tuples[j];
        if (t.is_null()) {
            for (size_t d = 0; d < cells.cols(); ++d) {
                cells(j, d) = 0;
            }
        } else {
            int64_t vid = mesh.id(t, PrimitiveType::Vertex);
            cells(j, 0) = vid;

            auto t1 = mesh.switch_tuple(t, PrimitiveType::Vertex);

            cells(j, 1) = mesh.id(t1, PrimitiveType::Vertex);

            t1 = mesh.switch_tuple(t, PrimitiveType::Edge);
            auto t2 = mesh.switch_tuple(t1, PrimitiveType::Vertex);

            cells(j, 2) = mesh.id(t2, PrimitiveType::Vertex);
        }
    }
    m_writer.init(filename.string() + ".obj", vertices_name, cells);
}


void ObjWriter::ObjInternalWriter::init(
    const std::filesystem::path& filename,
    const std::string& vertices_name,
    const Eigen::MatrixXi& elements)
{
    m_vertices_name = vertices_name;
    m_elements = elements;

    m_filename = filename;
}


void ObjWriter::ObjInternalWriter::write(
    const std::string& name,
    const int64_t stride,
    const std::vector<double>& val)
{
    logger().warn("the name is {}", name);

    if (name == m_vertices_name) {
        logger().critical("the name is {}", name);
        assert(stride == 2 || stride == 3);

        Eigen::MatrixXd V =
            Eigen::Map<const Eigen::MatrixXd>(&val[0], stride, val.size() / stride).transpose();

        std::ofstream s(m_filename);
        if (!s.is_open()) {
            fprintf(stderr, "IOError: writeOBJ() could not open %s\n", name.c_str());
            return;
        }
        s << V.format(Eigen::IOFormat(
                 Eigen::FullPrecision,
                 Eigen::DontAlignCols,
                 " ",
                 "\n",
                 "v ",
                 "",
                 "",
                 "\n"))
          << (m_elements.array() + 1)
                 .format(Eigen::IOFormat(
                     Eigen::FullPrecision,
                     Eigen::DontAlignCols,
                     " ",
                     "\n",
                     "f ",
                     "",
                     "",
                     "\n"));


    } else
        return;
}

void ObjWriter::write(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<double>& val,
    double default_value)
{
    logger().critical("is it called?");
    m_writer.write(name, stride, val);
}
void ObjWriter::write(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<int64_t>& val,
    const int64_t default_val)
{
    return;
}
void ObjWriter::write(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<char>& val,
    const char default_val)
{
    return;
}
void ObjWriter::write(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<Rational>& val,
    const Rational& default_val)
{
    return;
}
bool ObjWriter::write(const int dim)
{
    return true;
}
void ObjWriter::write_top_simplex_type(const PrimitiveType type)
{
    return;
}

void ObjWriter::write_absolute_id(const std::vector<int64_t>& id)
{
    return;
}

void ObjWriter::write_capacities(const std::vector<int64_t>& capacities)
{
    return;
}

} // namespace wmtk::io