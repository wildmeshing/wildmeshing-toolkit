#include "HDF5Reader.hpp"

#include <wmtk/Mesh.hpp>

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>

#include <h5pp/h5pp.h>

#include <regex>


namespace wmtk {
HDF5Reader::HDF5Reader() {}


std::shared_ptr<Mesh> HDF5Reader::read(const std::filesystem::path& filename)
{
    h5pp::File m_hdf5_file(filename, h5pp::FileAccess::READONLY);

    PrimitiveType top_simplex_type =
        m_hdf5_file.readAttribute<PrimitiveType>("WMTK", "top_simplex_type");

    std::shared_ptr<Mesh> mesh;

    switch (top_simplex_type) {
    case PrimitiveType::Vertex: mesh = std::make_shared<PointMesh>(); break;
    case PrimitiveType::HalfEdge:
    case PrimitiveType::Edge: mesh = std::make_shared<EdgeMesh>(); break;
    case PrimitiveType::Face: mesh = std::make_shared<TriMesh>(); break;
    case PrimitiveType::Tetrahedron: mesh = std::make_shared<TetMesh>(); break;
    default: break;
    }

    std::vector<int64_t> capacities =
        m_hdf5_file.readAttribute<std::vector<int64_t>>("WMTK", "capacities");


    mesh->set_capacities(capacities);

    const auto dsets = m_hdf5_file.findDatasets("", "WMTK");
    for (auto& s : dsets) {
        const std::string dataset = "WMTK/" + s;
        const int64_t stride = m_hdf5_file.readAttribute<int64_t>(dataset, "stride");
        const int64_t dimension = m_hdf5_file.readAttribute<int64_t>(dataset, "dimension");
        const std::string type = m_hdf5_file.readAttribute<std::string>(dataset, "type");
        const std::string name =
            std::regex_replace(s, std::regex(std::to_string(dimension) + "/"), "");

        auto pt = PrimitiveType(dimension);

        if (type == "int64_t") {
            auto v = m_hdf5_file.readDataset<std::vector<int64_t>>(dataset);
            set_attribute<int64_t>(name, pt, stride, v, *mesh);
        } else if (type == "char") {
            auto tmp = m_hdf5_file.readDataset<std::vector<short>>(dataset);
            std::vector<char> v;
            v.reserve(tmp.size());
            for (auto val : tmp) v.push_back(char(val));

            set_attribute<char>(name, pt, stride, v, *mesh);
        } else if (type == "double") {
            auto v = m_hdf5_file.readDataset<std::vector<double>>(dataset);

            set_attribute<double>(name, pt, stride, v, *mesh);
        } else if (type == "rational") {
            auto tmp = m_hdf5_file.readDataset<std::vector<std::string>>(dataset);
            assert(tmp.size() % 2 == 0);

            std::vector<Rational> v;
            v.reserve(tmp.size() / 2);
            for (size_t i = 0; i < tmp.size(); i += 2) {
                v.emplace_back(tmp[i], tmp[i + 1]);
            }

            set_attribute<Rational>(name, pt, stride, v, *mesh);

        } else {
            logger().error("We currently do not support reading the type \"{}\"", type);
            assert(false);
        }
    }

    return mesh;
}

template <typename T>
void HDF5Reader::set_attribute(
    const std::string& name,
    PrimitiveType pt,
    int64_t stride,
    const std::vector<T>& v,
    Mesh& mesh)
{
    MeshAttributeHandle handle = mesh.has_attribute<T>(name, pt)
                                        ? mesh.get_attribute_handle<T>(name, pt)
                                        : mesh.register_attribute<T>(name, pt, stride);

    if (stride != mesh.attribute_dimension(handle.handle)) {
        log_and_throw_error(
            "Attribute does not have the expected dimension:\n  expected {}\n  actual {}",
            stride,
            handle.dimension());
    }

    auto accessor = attribute::AccessorBase<T>(mesh, handle);

    accessor.set_attribute(v);
}


} // namespace wmtk
