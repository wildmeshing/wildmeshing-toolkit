#include "AsdFunction.hpp"

#include <wmtk/Mesh.hpp>

namespace wmtk::function {

AsdFunction::AsdFunction(
    Mesh& mesh,
    MeshAttributeHandle<double>& handle,
    const simplex::Simplex& simplex)
    : m_mesh(mesh)
    , m_handle(handle)
    , m_accessor(mesh.create_accessor(handle))
    , m_simplex(simplex)
{}

long AsdFunction::embedded_dimension() const
{
    return mesh().get_attribute_dimension(m_handle);
}

double AsdFunction::value(const TVector& x)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    double res = get_value(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;

    return res;
}

void AsdFunction::gradient(const TVector& x, TVector& gradv)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    gradv = get_gradient(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;
}

void AsdFunction::hessian(const TVector& x, Eigen::MatrixXd& hessian)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    hessian = get_hessian(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;
}

void AsdFunction::solution_changed(const TVector& new_x)
{
    m_accessor.vector_attribute(m_simplex.tuple()) = new_x;
}


bool AsdFunction::is_step_valid(const TVector& x0, const TVector& x1) const
{
    // TODO use invariants
    return true;
}

} // namespace wmtk::function
