#include "MeshWriter.hpp"

#include <wmtk/utils/Rational.hpp>

#include <h5pp/h5pp.h>

#include <sstream>

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
    std::stringstream ss;
    ss << "WMTK/" << std::to_string(type) << "/" << name;

    m_hdf5_file->writeDataset(val, ss.str());
    m_hdf5_file->writeAttribute(stride, ss.str(), "stride");
    m_hdf5_file->writeAttribute(type, ss.str(), "type");
}

template <>
void MeshWriter::write<char>(
    const std::string& name,
    const long type,
    const long stride,
    const std::vector<char>& val)
{
    std::vector<short> tmp;
    tmp.reserve(val.size());
    for (const auto& v : val) tmp.push_back(v);

    std::stringstream ss;
    ss << "WMTK/" << std::to_string(type) << "/" << name;

    m_hdf5_file->writeDataset(tmp, ss.str());
    m_hdf5_file->writeAttribute(stride, ss.str(), "stride");
    m_hdf5_file->writeAttribute(type, ss.str(), "type");
}

template <>
void MeshWriter::write<Rational>(
    const std::string& name,
    const long type,
    const long stride,
    const std::vector<Rational>& val)
{
    std::vector<std::string> tmp;
    tmp.reserve(val.size());
    for (const auto& v : val) {
        tmp.emplace_back(v.numerator() + "/" + v.denominator());
    }

    std::stringstream ss;
    ss << "WMTK/" << std::to_string(type) << "/" << name;

    m_hdf5_file->writeDataset(tmp, ss.str());
    m_hdf5_file->writeAttribute(stride, ss.str(), "stride");
    m_hdf5_file->writeAttribute(type, ss.str(), "type");
}


template void
MeshWriter::write<long>(const std::string&, const long, const long, const std::vector<long>&);
template void
MeshWriter::write<double>(const std::string&, const long, const long, const std::vector<double>&);


} // namespace wmtk