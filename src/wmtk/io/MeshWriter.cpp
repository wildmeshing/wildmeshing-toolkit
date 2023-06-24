#include "MeshWriter.hpp"

#include <h5pp/h5pp.h>

namespace wmtk {
MeshWriter::MeshWriter(const std::filesystem::path& filename)
{
    m_hdf5_file = std::make_shared<h5pp::File>(filename, h5pp::FileAccess::REPLACE);
}

template <typename T>
void MeshWriter::write(
    const std::string& name,
    const long type,
    const long stride,
    const std::vector<T>& val)
{
    m_hdf5_file->writeDataset(std::array<const std::vector<T>, 1>{{val}}, "WMTK/" + name);
    m_hdf5_file->writeAttribute(stride, "WMTK/" + name, "stride");
    m_hdf5_file->writeAttribute(type, "WMTK/" + name, "type");
}


template void
MeshWriter::write<char>(const std::string&, const long, const long, const std::vector<char>&);
template void
MeshWriter::write<long>(const std::string&, const long, const long, const std::vector<long>&);
template void
MeshWriter::write<double>(const std::string&, const long, const long, const std::vector<double>&);

} // namespace wmtk