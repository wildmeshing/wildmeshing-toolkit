#pragma once
#include "autodiff.h"

namespace wmtk::function::utils {


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
    int rows = data.rows();
    int cols = data.cols();

    assert(rows * cols == DiffScalarBase::getVariableCount());

    using RetType = Eigen::Matrix<DScalarType, Rows, Cols>;
    if constexpr (Rows != Eigen::Dynamic && Cols != Eigen::Dynamic) {
        return RetType::NullaryExpr([&](int row, int col) {
                   int index;
                   if constexpr (RetType::IsRowMajor) {
                       index = Rows * col + row;
                   } else {
                       index = Cols * row + col;
                   }
                   return DScalarType(index, data(row, col));
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
                       return DScalarType(index, data(row, col));
                   })
            .eval();
    }
}


} // namespace wmtk::function::utils
