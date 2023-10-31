#include "HDF5Writer.hpp"

#include <wmtk/utils/Rational.hpp>

#include <h5pp/h5pp.h>

#include <sstream>

namespace wmtk {
namespace {
template <typename T>
std::string get_type()
{
    return "";
}

template <>
std::string get_type<long>()
{
    return "long";
}

template <>
std::string get_type<double>()
{
    return "double";
}

template <>
std::string get_type<short>()
{
    return "char";
}

template <>
std::string get_type<std::array<std::string, 2>>()
{
    return "rational";
}
} // namespace

HDF5Writer::HDF5Writer(const std::filesystem::path& filename)
{
    m_hdf5_file = std::make_shared<h5pp::File>(filename, h5pp::FileAccess::REPLACE);
}

void HDF5Writer::write(
    const std::string& name,
    const long type,
    const long stride,
    const std::vector<long>& val)
{
    write_internal(name, type, stride, val);
}

void HDF5Writer::write(
    const std::string& name,
    const long type,
    const long stride,
    const std::vector<double>& val)
{
    write_internal(name, type, stride, val);
}

void HDF5Writer::write(
    const std::string& name,
    const long type,
    const long stride,
    const std::vector<char>& val)
{
    std::vector<short> tmp;
    tmp.reserve(val.size());
    for (const auto& v : val) tmp.push_back(v);

    write_internal(name, type, stride, tmp);
}


void HDF5Writer::write(
    const std::string& name,
    const long type,
    const long stride,
    const std::vector<Rational>& val)
{
    std::vector<std::array<std::string, 2>> tmp;
    tmp.reserve(val.size());
    for (const auto& v : val) {
        tmp.push_back({{v.numerator(), v.denominator()}});
    }

    write_internal(name, type, stride, tmp);
}

void HDF5Writer::write_capacities(const std::vector<long>& capacities)
{
    m_hdf5_file->writeAttribute(capacities, "WMTK", "capacities");
}

template <typename T>
void HDF5Writer::write_internal(
    const std::string& name,
    const long type,
    const long stride,
    const std::vector<T>& val)
{
    std::stringstream ss;
    ss << "WMTK/" << type << "/" << name;

    m_hdf5_file->writeDataset(val, ss.str());
    m_hdf5_file->writeAttribute(stride, ss.str(), "stride");
    m_hdf5_file->writeAttribute(type, ss.str(), "dimension");
    m_hdf5_file->writeAttribute("rational", ss.str(), "type");
    m_hdf5_file->writeAttribute(get_type<T>(), ss.str(), "type");
}

void HDF5Writer::write_top_simplex_type(const PrimitiveType type)
{
    m_hdf5_file->createGroup("WMTK");
    m_hdf5_file->writeAttribute(type, "WMTK", "top_simplex_type");
}


} // namespace wmtk
