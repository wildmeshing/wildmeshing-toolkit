#pragma once
#include "autodiff.h"

namespace wmtk::function {


template <typename DScalarType, int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
auto make_DScalar_matrix(int rows = 0, int cols = 0)
{
    if constexpr (Rows != Eigen::Dynamic) {
        rows = Rows;
    }
    if constexpr (Cols != Eigen::Dynamic) {
        cols = Cols;
    }
    assert(rows * cols == DiffScalarBase::getVariableCount());

    using RetType = Eigen::Matrix<DScalarType, Rows, Cols>;
    if constexpr (Rows != Eigen::Dynamic && Cols != Eigen::Dynamic) {
        return RetType::NullaryExpr([](int row, int col) {
                   int index;
                   if constexpr (RetType::IsRowMajor) {
                       index = Rows * col + row;
                   } else {
                       index = Cols * row + col;
                   }
                   return DScalarType(index);
               })
            .eval();
    } else {
        return RetType::NullaryExpr(
                   rows,
                   cols,
                   [&](int row, int col) {
                       int index;
                       if constexpr (RetType::IsRowMajor) {
                           index = rows * col + row;
                       } else {
                           index = cols * row + col;
                       }
                       return DScalarType(index);
                   })
            .eval();
    }
}

template <typename DScalarType, typename Derived>
auto as_DScalar(const Eigen::MatrixBase<Derived>& data)
{
    constexpr static int Rows = Derived::RowsAtCompileTime;
    constexpr static int Cols = Derived::ColsAtCompileTime;

    Eigen::Matrix<DScalarType, Rows, Cols> M =
        make_DScalar_matrix<DScalarType, Rows, Cols>(data.rows(), data.cols());

    M.noalias() = M.binaryExpr(data, [](DScalarType v, const auto& d) {
        v = d;
        return v;
    });


    return M;
}


} // namespace wmtk::function
