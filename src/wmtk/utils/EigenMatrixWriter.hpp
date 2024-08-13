#pragma once

#include <map>
#include <wmtk/Types.hpp>
#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::utils {
class EigenMatrixWriter : public MeshWriter
{
public:
    EigenMatrixWriter() = default;
    ~EigenMatrixWriter() = default;

    void get_position_matrix(MatrixX<double>& matrix);
    void get_position_matrix(MatrixX<Rational>& matrix);
    void get_TV_matrix(MatrixX<int64_t>& matrix);
    void get_FV_matrix(MatrixX<int64_t>& matrix);
    void get_EV_matrix(MatrixX<int64_t>& matrix);

    void
    get_double_matrix(const std::string& name, const PrimitiveType type, MatrixX<double>& matrix);
    void
    get_int64_t_matrix(const std::string& name, const PrimitiveType type, MatrixX<int64_t>& matrix);
    void get_char_matrix(const std::string& name, const PrimitiveType type, MatrixX<char>& matrix);
    void get_Rational_matrix(
        const std::string& name,
        const PrimitiveType type,
        MatrixX<Rational>& matrix);

    /*These are useless, just override*/
    void write_top_simplex_type(const PrimitiveType type) override;
    void write_absolute_id(const std::vector<int64_t>& id) override;
    void write_capacities(const std::vector<int64_t>& capacities) override;
    bool write(const int dim) override { return true; }
    /**/

    void write(
        const std::string& name,
        const int64_t type,
        const int64_t stride,
        const std::vector<char>& val,
        const char default_val) override;

    void write(
        const std::string& name,
        const int64_t type,
        const int64_t stride,
        const std::vector<int64_t>& val,
        const int64_t default_val) override;

    void write(
        const std::string& name,
        const int64_t type,
        const int64_t stride,
        const std::vector<double>& val,
        const double default_val) override;

    void write(
        const std::string& name,
        const int64_t type,
        const int64_t stride,
        const std::vector<Rational>& val,
        const Rational& default_val) override;

private:
    std::map<std::pair<std::string, PrimitiveType>, MatrixX<double>> doubles;
    std::map<std::pair<std::string, PrimitiveType>, MatrixX<int64_t>> int64_ts;
    std::map<std::pair<std::string, PrimitiveType>, MatrixX<char>> chars;
    std::map<std::pair<std::string, PrimitiveType>, MatrixX<Rational>> Rationals;

    template <typename T>
    void write_internal(
        std::map<std::pair<std::string, PrimitiveType>, MatrixX<T>>& Ts,
        const std::string& name,
        const int64_t type,
        const int64_t stride,
        const std::vector<T>& val,
        const T& default_val);
};

} // namespace wmtk::utils
