#if defined(__GNUG__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
#include <string>
#if defined(__GNUG__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
#include <fmt/format.h>
#include <fmt/ranges.h>
#include "HDF5Writer.hpp"

#include <wmtk/utils/Rational.hpp>

#include <h5pp/h5pp.h>

#include <sstream>

namespace wmtk {
namespace {
template <typename T>
std::string get_type()
{
    assert(false);
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
std::string get_type<char>()
{
    return "char";
}
template <>
std::string get_type<short>()
{
    return "char";
}

template <>
std::string get_type<wmtk::Rational>()
{
    return "rational";
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
    tmp.reserve(val.size());
    int64_t max_size = -1;
    for (const auto& v : val) {
        tmp.emplace_back(v.serialize());
        max_size = std::max(max_size, int64_t(tmp.back().size()));
    }

    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data(tmp.size(), max_size);
    data.setConstant(int(' '));
    for (int64_t i = 0; i < data.rows(); ++i) {
        for (int64_t j = 0; j < tmp[i].size(); ++j) {
            data(i, j) = int(tmp[i][j]);
        }
    }

    write_internal(name, type, stride, data, default_val.to_binary());
}

void HDF5Writer::write_capacities(const std::vector<int64_t>& capacities)
{
    m_hdf5_file->writeAttribute(capacities, dataset_path(), "capacities");
}

template <typename Data, typename T>
void HDF5Writer::write_internal(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const Data& val,
    const T& default_val)
{
    std::string path = fmt::format("{}/{}/{}", dataset_path(), type, name);

    m_hdf5_file->writeDataset(val, path);
    m_hdf5_file->writeAttribute(stride, path, "stride");
    m_hdf5_file->writeAttribute(default_val, path, "default_value");
    m_hdf5_file->writeAttribute(type, path, "dimension");
    m_hdf5_file->writeAttribute(get_type<T>(), path, "type");
}

void HDF5Writer::write_top_simplex_type(const PrimitiveType type)
{
    m_hdf5_file->writeAttribute(type, dataset_path(), "top_simplex_type");
}

void HDF5Writer::write_absolute_id(const std::vector<int64_t>& id)
{
    if (id.empty() || m_mm_level == 0) {
        m_name = "";
    } else {
        m_name = fmt::format("mesh_{}", fmt::join(id, ""));
    }

    ++m_mm_level;

    m_hdf5_file->createGroup(dataset_path());

    if (!id.empty()) m_hdf5_file->writeAttribute(id, dataset_path(), "absolute_id");
}

std::string HDF5Writer::dataset_path() const
{
    std::string res = "WMTK";

    if (!m_name.empty()) res += "/multimesh/" + m_name;

    return res;
}


template <typename T>
void HDF5Writer::write_attribute_names(int dim, const std::vector<std::string>& names)
{
    if (names.empty()) {
        return;
    }
    const static std::string& name = get_type<T>();
    const std::string path =
        fmt::format("{}/{}_{}/{}", dataset_path(), "ATTRIBUTE_LIST", name, dim);

    m_hdf5_file->writeDataset(names, path);
}

template void HDF5Writer::write_attribute_names<double>(int dim, const std::vector<std::string>&);
template void HDF5Writer::write_attribute_names<int64_t>(int dim, const std::vector<std::string>&);
template void HDF5Writer::write_attribute_names<char>(int dim, const std::vector<std::string>&);
template void HDF5Writer::write_attribute_names<wmtk::Rational>(
    int dim,
    const std::vector<std::string>&);
} // namespace wmtk
