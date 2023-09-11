#include "MeshReader.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>

#include <h5pp/h5pp.h>

#include <regex>


namespace wmtk {
MeshReader::MeshReader(const std::filesystem::path& filename)
    : m_filename(filename)
{}


void MeshReader::read(Mesh& mesh)
{
    h5pp::File m_hdf5_file(m_filename, h5pp::FileAccess::READONLY);

    std::vector<long> capacities =
        m_hdf5_file.readAttribute<std::vector<long>>("WMTK", "capacities");

    mesh.set_capacities(capacities);

    const auto dsets = m_hdf5_file.findDatasets("", "WMTK");
    for (auto& s : dsets) {
        const std::string dataset = "WMTK/" + s;
        const long stride = m_hdf5_file.readAttribute<long>(dataset, "stride");
        const long dimension = m_hdf5_file.readAttribute<long>(dataset, "dimension");
        const std::string type = m_hdf5_file.readAttribute<std::string>(dataset, "type");
        const std::string name =
            std::regex_replace(s, std::regex(std::to_string(dimension) + "/"), "");

        auto pt = PrimitiveType(dimension);

        if (type == "long") {
            auto v = m_hdf5_file.readDataset<std::vector<long>>(dataset);
            set_attribute<long>(name, pt, stride, v, mesh);
        } else if (type == "char") {
            auto tmp = m_hdf5_file.readDataset<std::vector<short>>(dataset);
            std::vector<char> v;
            v.reserve(tmp.size());
            for (auto val : tmp) v.push_back(char(val));

            set_attribute<char>(name, pt, stride, v, mesh);
        } else if (type == "double") {
            auto v = m_hdf5_file.readDataset<std::vector<double>>(dataset);

            set_attribute<double>(name, pt, stride, v, mesh);
        } else if (type == "rational") {
            logger().error("We currently do not support reading rationals");
            assert(false); //
            auto tmp = m_hdf5_file.readDataset<std::vector<std::array<std::string, 2>>>(dataset);

            std::vector<Rational> v;
            v.reserve(tmp.size());
            for (auto val : tmp) v.emplace_back(val[0], val[1]);

            // set_attribute<Rational>(name, pt, stride, v, mesh);

        } else {
            logger().error("We currently do not support reading the type \"{}\"", type);
            assert(false);
        }
    }
}

template <typename T>
void MeshReader::set_attribute(
    const std::string& name,
    PrimitiveType pt,
    long stride,
    const std::vector<T>& v,
    Mesh& mesh)
{
    auto handle = mesh.register_attribute<T>(name, pt, stride, true);
    auto accessor = attribute::AccessorBase<T>(mesh, handle);

    accessor.set_attribute(v);
}


} // namespace wmtk
