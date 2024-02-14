#include "EigenMatrixWriter.hpp"

namespace wmtk::utils {

void EigenMatrixWriter::get_position_matrix(MatrixX<double>& matrix)
{
    get_double_matrix("vertices", PrimitiveType::Vertex, matrix);
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

void EigenMatrixWriter::get_double_matrix(
    const std::string& name,
    const PrimitiveType type,
    MatrixX<double>& matrix)
{
    if (doubles.find(std::make_pair(name, type)) != doubles.end()) {
        matrix = doubles[std::make_pair(name, type)];
    } else {
        throw std::runtime_error("No attribute named " + name);
    }
}

void EigenMatrixWriter::get_int64_t_matrix(
    const std::string& name,
    const PrimitiveType type,
    MatrixX<int64_t>& matrix)
{
    if (int64_ts.find(std::make_pair(name, type)) != int64_ts.end()) {
        matrix = int64_ts[std::make_pair(name, type)];
    } else {
        throw std::runtime_error("No attribute named " + name);
    }
}

void EigenMatrixWriter::get_char_matrix(
    const std::string& name,
    const PrimitiveType type,
    MatrixX<char>& matrix)
{
    if (chars.find(std::make_pair(name, type)) != chars.end()) {
        matrix = chars[std::make_pair(name, type)];
    } else {
        throw std::runtime_error("No attribute named " + name);
    }
}

void EigenMatrixWriter::get_Rational_matrix(
    const std::string& name,
    const PrimitiveType type,
    MatrixX<Rational>& matrix)
{
    if (Rationals.find(std::make_pair(name, type)) != Rationals.end()) {
        matrix = Rationals[std::make_pair(name, type)];
    } else {
        throw std::runtime_error("No attribute named " + name);
    }
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


} // namespace wmtk::utils