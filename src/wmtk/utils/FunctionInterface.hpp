#pragma once
#include <Eigen/Core>
#include <wmtk/Accessor.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/function/DifferentiableFunction.hpp>

namespace wmtk {
class FunctionInterface
{
public:
    FunctionInterface(
        const Tuple& tuple,
        Accessor<double>& accessor,
        const function::DifferentiableFunction& function)
        : m_tuple(tuple)
        , m_accessor(accessor)
        , m_function(function)
    {}

    const Tuple& m_tuple;
    Accessor<double>& m_accessor;
    const function::DifferentiableFunction& m_function;

    void store(Eigen::Vector2d& v) { m_accessor.vector_attribute(m_tuple) = v; }
    double eval() const { return m_function.get_value(m_tuple); }
    Eigen::Vector2d grad() const { return m_function.get_gradient(m_tuple); }
    Eigen::Matrix2d hess() const { return m_function.get_hessian(m_tuple); }
};
} // namespace wmtk
