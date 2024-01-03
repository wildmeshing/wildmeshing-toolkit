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
std::string get_type<int64_t>()
{
    return "int64_t";
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
std::string get_type<std::string>()
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
    const int64_t type,
    const int64_t stride,
    const std::vector<int64_t>& val,
    const int64_t default_val)
{
    write_internal(name, type, stride, val, default_val);
}

void HDF5Writer::write(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<double>& val,
    const double default_val)
{
    write_internal(name, type, stride, val, default_val);
}

void HDF5Writer::write(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<char>& val,
    const char default_val)
{
    std::vector<short> tmp;
    tmp.reserve(val.size());
    for (const auto& v : val) tmp.push_back(v);

    write_internal(name, type, stride, tmp, short(default_val));
}


void HDF5Writer::write(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<Rational>& val,
    const Rational& default_val)
{
    std::vector<std::string> tmp;
    tmp.reserve(val.size() * 2);
    for (const auto& v : val) {
        tmp.emplace_back(v.numerator());
        tmp.emplace_back(v.denominator());
    }
    std::stringstream ss;
    ss << default_val;
    write_internal(name, type, stride, tmp, ss.str());
}

void HDF5Writer::write_capacities(const std::vector<int64_t>& capacities)
{
    m_hdf5_file->writeAttribute(capacities, dataset_path(), "capacities");
}

template <typename T>
void HDF5Writer::write_internal(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<T>& val,
    const T& default_val)
{
    std::stringstream ss;
    ss << dataset_path() << "/" << type << "/" << name;

    m_hdf5_file->writeDataset(val, ss.str());
    m_hdf5_file->writeAttribute(stride, ss.str(), "stride");
    m_hdf5_file->writeAttribute(default_val, ss.str(), "default_value");
    m_hdf5_file->writeAttribute(type, ss.str(), "dimension");
    m_hdf5_file->writeAttribute(get_type<T>(), ss.str(), "type");
}

void HDF5Writer::write_top_simplex_type(const PrimitiveType type)
{
    m_hdf5_file->writeAttribute(type, dataset_path(), "top_simplex_type");
}

void HDF5Writer::write_absolute_id(const std::vector<int64_t>& id)
{
    if (id.empty()) {
        m_name = "";
    } else {
        m_name = fmt::format("mesh_{}", fmt::join(id, ""));
    }

    m_hdf5_file->createGroup(dataset_path());

    if (!id.empty()) m_hdf5_file->writeAttribute(id, dataset_path(), "absolute_id");
}

std::string HDF5Writer::dataset_path() const
{
    std::string res = "WMTK";

    if (!m_name.empty()) res += "/multimesh/" + m_name;

    return res;
}


} // namespace wmtk
