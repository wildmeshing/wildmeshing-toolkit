#pragma once
#include <Eigen/Core>
#include <wmtk/Accessor.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/function/DifferentiableFunction.hpp>

namespace wmtk::optimization {
template <int Dim>
class FunctionInterface
{
public:
    using Vector = wmtk::Vector<double, Dim>;
    using Matrix = wmtk::SquareMatrix<double, Dim>;
    FunctionInterface(
        const Tuple& tuple,
        Accessor<double>& accessor,
        const function::DifferentiableFunction& function)
        : m_tuple(tuple)
        , m_accessor(accessor)
        , m_function(function)
    {}


    auto get_coordinate() { return m_accessor.vector_attribute(m_tuple); }
    auto get_const_coordinate() const { return m_accessor.const_vector_attribute(m_tuple); }
    template <typename Derived>
    void store(const Eigen::MatrixBase<Derived>& v)
    {
        m_accessor.vector_attribute(m_tuple) = v;
    }
    auto get_coordinate() const { return get_const_coordinate(); }

    double get_value() const { return m_function.get_value(m_tuple); }
    Vector get_gradient() const { return m_function.get_gradient(m_tuple); }
    Matrix get_hessian() const { return m_function.get_hessian(m_tuple); }

    template <typename Derived>
    double get_value(const Eigen::MatrixBase<Derived>& v)
    {
        store(v);
        return get_value();
    }
    template <typename Derived>
    Vector get_gradient(const Eigen::MatrixBase<Derived>& v)
    {
        store(v);
        return get_gradient();
    }
    template <typename Derived>
    Matrix get_hessian(const Eigen::MatrixBase<Derived>& v)
    {
        store(v);
        return get_hessian();
    }


    const Tuple& tuple() const { return m_tuple; }
    Mesh& mesh() { return m_accessor.mesh(); }

private:
    const Tuple& m_tuple;
    Accessor<double>& m_accessor;
    const function::DifferentiableFunction& m_function;
};
} // namespace wmtk::optimization
