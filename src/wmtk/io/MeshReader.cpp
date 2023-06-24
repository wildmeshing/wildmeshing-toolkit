#include "MeshReader.hpp"

#include <wmtk/utils/Rational.hpp>

#include <h5pp/h5pp.h>

#include <regex>


namespace wmtk {
MeshReader::MeshReader(const std::filesystem::path& filename)
{
    h5pp::File m_hdf5_file(filename, h5pp::FileAccess::READONLY);

    const auto dsets = m_hdf5_file.findDatasets("", "WMTK");
    for (auto& s : dsets) {
        const auto dataset = "WMTK/" + s;
        const long stride = m_hdf5_file.readAttribute<long>(dataset, "stride");
        const long dimension = m_hdf5_file.readAttribute<long>(dataset, "dimension");
        const std::string type = m_hdf5_file.readAttribute<std::string>(dataset, "type");
        const std::string name =
            std::regex_replace(s, std::regex(std::to_string(dimension) + "/"), "");

        if (type == "long") {
            auto v = m_hdf5_file.readDataset<std::vector<long>>(dataset);
        } else if (type == "char") {
            auto tmp = m_hdf5_file.readDataset<std::vector<short>>(dataset);
            std::vector<char> v;
            v.reserve(tmp.size());
            for (auto val : tmp) v.push_back(char(val));
        } else if (type == "double") {
            auto v = m_hdf5_file.readDataset<std::vector<double>>(dataset);
        } else if (type == "rational") {
            auto tmp = m_hdf5_file.readDataset<std::vector<std::array<std::string, 2>>>(dataset);

            std::vector<Rational> v;
            v.reserve(tmp.size());
            for (auto val : tmp) v.emplace_back(val[0], val[1]);
        } else {
            assert(false);
        }

        std::cout << name << " " << stride << " " << type << std::endl;
    }
}
} // namespace wmtk