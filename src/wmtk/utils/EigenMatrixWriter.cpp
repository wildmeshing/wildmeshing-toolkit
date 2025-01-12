#include "EigenMatrixWriter.hpp"
#include <fmt/format.h>
#include <typeinfo>

namespace wmtk::utils {

void EigenMatrixWriter::set_position_attribute_name(const std::string_view& name)
{
    m_position_attribute_name = name;
}
void EigenMatrixWriter::get_position_matrix(MatrixX<double>& matrix)
{
    get_double_matrix(m_position_attribute_name, PrimitiveType::Vertex, matrix);
}

void EigenMatrixWriter::get_position_matrix(MatrixX<Rational>& matrix)
{
    get_Rational_matrix(m_position_attribute_name, PrimitiveType::Vertex, matrix);
}


void EigenMatrixWriter::get_TV_matrix(MatrixX<int64_t>& matrix)
{
    get_int64_t_matrix("m_tv", PrimitiveType::Tetrahedron, matrix);
}

void EigenMatrixWriter::get_FV_matrix(MatrixX<int64_t>& matrix)
{
    get_int64_t_matrix("m_fv", PrimitiveType::Triangle, matrix);
}

void EigenMatrixWriter::get_EV_matrix(MatrixX<int64_t>& matrix)
{
    get_int64_t_matrix("m_ev", PrimitiveType::Edge, matrix);
}

auto EigenMatrixWriter::get_simplex_vertex_matrix() const -> MatrixXl
{
    const static std::array<std::pair<std::string, PrimitiveType>, 3> keys = {
        {std::make_pair("m_tv", PrimitiveType::Tetrahedron),
         std::make_pair("m_fv", PrimitiveType::Triangle),
         std::make_pair("m_ev", PrimitiveType::Edge)}};
    for (const auto& [n, pt] : keys) {
        try {
            return get_matrix<int64_t>(n, pt);
        } catch (const std::out_of_range& e) {
            continue;
        }
    }
    assert(false);
    return {};
}

template <typename T>
bool EigenMatrixWriter::has_matrix(const std::string& name, const PrimitiveType type) const
{
    try {
        get_matrix<T>(name, type);
    } catch (const std::out_of_range& e) {
        return false;
    }
    return true;
}

template <typename T>
Eigen::MatrixX<T> EigenMatrixWriter::get_matrix(const std::string& name, const PrimitiveType type)
    const
{
    auto pair = std::make_pair(name, type);
    try {
        if constexpr (std::is_same_v<T, char>) {
            return chars.at(pair);
        } else if constexpr (std::is_same_v<T, double>) {
            return doubles.at(pair);
        } else if constexpr (std::is_same_v<T, int64_t>) {
            return int64_ts.at(pair);
        } else if constexpr (std::is_same_v<T, Rational>) {
            return Rationals.at(pair);
        }
    } catch (const std::out_of_range& e) {
        throw std::out_of_range(fmt::format(
            "No attribute named {} with primitive {} found on {}",
            name,
            primitive_type_name(type),
            typeid(T).name()));
    }
    return {};
}

void EigenMatrixWriter::get_double_matrix(
    const std::string& name,
    const PrimitiveType type,
    MatrixX<double>& matrix)
{
    matrix = get_matrix<double>(name, type);
}

void EigenMatrixWriter::get_int64_t_matrix(
    const std::string& name,
    const PrimitiveType type,
    MatrixX<int64_t>& matrix)
{
    matrix = get_matrix<int64_t>(name, type);
}

void EigenMatrixWriter::get_char_matrix(
    const std::string& name,
    const PrimitiveType type,
    MatrixX<char>& matrix)
{
    matrix = get_matrix<char>(name, type);
}

void EigenMatrixWriter::get_Rational_matrix(
    const std::string& name,
    const PrimitiveType type,
    MatrixX<Rational>& matrix)
{
    matrix = get_matrix<Rational>(name, type);
}

template <typename T>
void EigenMatrixWriter::write_internal(
    std::map<std::pair<std::string, PrimitiveType>, MatrixX<T>>& Ts,
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<T>& val,
    const T& default_val)
{
    if (m_mm_level != 1) return;
    using MapType = typename MatrixX<T>::ConstMapType;
    Ts[std::make_pair(name, PrimitiveType(type))] =
        MapType(val.data(), stride, val.size() / stride).transpose();
}

void EigenMatrixWriter::write(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<char>& val,
    const char default_val)
{
    write_internal(chars, name, type, stride, val, default_val);
}

void EigenMatrixWriter::write(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<int64_t>& val,
    const int64_t default_val)
{
    write_internal(int64_ts, name, type, stride, val, default_val);
}

void EigenMatrixWriter::write(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<double>& val,
    const double default_val)
{
    write_internal(doubles, name, type, stride, val, default_val);
}

void EigenMatrixWriter::write(
    const std::string& name,
    const int64_t type,
    const int64_t stride,
    const std::vector<Rational>& val,
    const Rational& default_val)
{
    write_internal(Rationals, name, type, stride, val, default_val);
}


/* use less functions below, TODO: should throw?*/
void EigenMatrixWriter::write_top_simplex_type(const PrimitiveType type)
{
    return;
}
void EigenMatrixWriter::write_absolute_id(const std::vector<int64_t>& id)
{
    ++m_mm_level;
}
void EigenMatrixWriter::write_capacities(const std::vector<int64_t>& capacities)
{
    return;
}

template Eigen::MatrixX<char> EigenMatrixWriter::get_matrix<char>(
    const std::string& name,
    const PrimitiveType type) const;
template Eigen::MatrixX<double> EigenMatrixWriter::get_matrix<double>(
    const std::string& name,
    const PrimitiveType type) const;
template Eigen::MatrixX<int64_t> EigenMatrixWriter::get_matrix<int64_t>(
    const std::string& name,
    const PrimitiveType type) const;
template Eigen::MatrixX<Rational> EigenMatrixWriter::get_matrix<Rational>(
    const std::string& name,
    const PrimitiveType type) const;

template bool EigenMatrixWriter::has_matrix<char>(const std::string& name, const PrimitiveType type)
    const;
template bool EigenMatrixWriter::has_matrix<double>(
    const std::string& name,
    const PrimitiveType type) const;
template bool EigenMatrixWriter::has_matrix<int64_t>(
    const std::string& name,
    const PrimitiveType type) const;
template bool EigenMatrixWriter::has_matrix<Rational>(
    const std::string& name,
    const PrimitiveType type) const;
} // namespace wmtk::utils
