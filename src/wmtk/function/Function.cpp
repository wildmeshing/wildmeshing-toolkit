#include "Function.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::function {

Function::Function(Mesh& mesh, MeshAttributeHandle<double>& handle, const simplex::Simplex& simplex)
    : m_mesh(mesh)
    , m_handle(handle)
    , m_accessor(mesh.create_accessor(handle))
    , m_simplex(simplex)
{}

long Function::embedded_dimension() const
{
    return mesh().get_attribute_dimension(m_handle);
}

double Function::value(const TVector& x)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    double res = get_value(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;

    return res;
}

void Function::gradient(const TVector& x, TVector& gradv)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    gradv = get_gradient(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;
}

void Function::hessian(const TVector& x, Eigen::MatrixXd& hessian)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    hessian = get_hessian(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;
}

void Function::solution_changed(const TVector& new_x)
{
    m_accessor.vector_attribute(m_simplex.tuple()) = new_x;
}


bool Function::is_step_valid(const TVector& x0, const TVector& x1) const
{
    // TODO use invariants
    return true;
}

} // namespace wmtk::function
