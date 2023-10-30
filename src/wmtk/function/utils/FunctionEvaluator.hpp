#pragma once
#include <Eigen/Core>
#include <wmtk/Accessor.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/function/Function.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::function::utils {


// Evaluates a function at a particular vertex of a mesh
// NOTE that this modifies attributes in the mesh.
// This should only be called from within a Scope so evaluates can be undone
//
class FunctionEvaluator
{
public:
    FunctionEvaluator(const Function& function, Accessor<double>& accessor, const Simplex& simplex);


    auto get_coordinate() { return m_accessor.vector_attribute(m_simplex.tuple()); }
    auto get_const_coordinate() const
    {
        return m_accessor.const_vector_attribute(m_simplex.tuple());
    }

    void store(double v);
    template <typename Derived>
    void store(const Eigen::MatrixBase<Derived>& v);

    auto get_coordinate() const { return get_const_coordinate(); }

    double get_value() const;

    template <typename Derived>
    double get_value(const Eigen::MatrixBase<Derived>& v);

    double get_value(double v);


    const Tuple& tuple() const { return m_simplex.tuple(); }
    const Simplex& simplex() const { return m_simplex; }
    Mesh& mesh() { return m_accessor.mesh(); }
    const Mesh& mesh() const { return m_accessor.mesh(); }
    Accessor<double>& accessor() { return m_accessor; }

    const Function& function() const { return m_function; }

    PrimitiveType my_simplex_type() const
    {
        PrimitiveType type = m_simplex.primitive_type();
        return type;
    }

private:
    const Function& m_function;
    Accessor<double>& m_accessor;
    const Simplex& m_simplex;
};


template <typename Derived>
void FunctionEvaluator::store(const Eigen::MatrixBase<Derived>& v)
{
    m_accessor.vector_attribute(tuple()) = v;
}
template <typename Derived>
double FunctionEvaluator::get_value(const Eigen::MatrixBase<Derived>& v)
{
    store(v);
    return get_value();
}
} // namespace wmtk::function::utils
