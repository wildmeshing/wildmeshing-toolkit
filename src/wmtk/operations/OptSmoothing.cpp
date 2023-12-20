#include "OptSmoothing.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/Mesh.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/Logger.hpp>

#include <polysolve/nonlinear/Solver.hpp>

namespace wmtk::operations {

OptSmoothing::WMTKProblem::WMTKProblem(
    Mesh& mesh,
    const MeshAttributeHandle<double>& handle,
    const simplex::Simplex& simplex,
    const std::unique_ptr<wmtk::function::Function>& energy)
    : m_handle(handle)
    , m_accessor(mesh.create_accessor(handle))
    , m_simplex(simplex)
    , m_energy(energy)
{}

OptSmoothing::WMTKProblem::TVector OptSmoothing::WMTKProblem::initial_value() const
{
    return m_accessor.vector_attribute(m_simplex.tuple());
}

double OptSmoothing::WMTKProblem::value(const TVector& x)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    double res = m_energy->get_value(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;

    return res;
}

void OptSmoothing::WMTKProblem::gradient(const TVector& x, TVector& gradv)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    gradv = m_energy->get_gradient(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;
}

void OptSmoothing::WMTKProblem::hessian(const TVector& x, Eigen::MatrixXd& hessian)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    hessian = m_energy->get_hessian(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;
}

void OptSmoothing::WMTKProblem::solution_changed(const TVector& new_x)
{
    m_accessor.vector_attribute(m_simplex.tuple()) = new_x;
}


bool OptSmoothing::WMTKProblem::is_step_valid(const TVector& x0, const TVector& x1) const
{
    // TODO use invariants
    return true;
}

void OperationSettings<OptSmoothing>::create_invariants()
{
    OperationSettings<AttributesUpdateBase>::create_invariants();
    // if (!smooth_boundary) {
    //     invariants->add(std::make_unique<InteriorVertexInvariant>(m_mesh));
    // }

    invariants->add(std::make_shared<TriangleInversionInvariant>(m_mesh, coordinate_handle));
}

OptSmoothing::OptSmoothing(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<OptSmoothing>& settings)
    : AttributesUpdateBase(m, t, settings)
    , m_settings{settings}
{}

std::string OptSmoothing::name() const
{
    return "opt_smoothing";
}

Accessor<double> OptSmoothing::coordinate_accessor()
{
    return mesh().create_accessor(m_settings.coordinate_handle);
}
ConstAccessor<double> OptSmoothing::const_coordinate_accessor() const
{
    return mesh().create_const_accessor(m_settings.coordinate_handle);
}

bool OptSmoothing::execute()
{
    if (!AttributesUpdateBase::execute()) return false;

    WMTKProblem problem(mesh(), coordinate_handle(), input_simplex(), m_settings.energy);

    polysolve::json linear_solver_params = R"({"solver": "Eigen::LDLT"})"_json;
    polysolve::json nonlinear_solver_params = R"({"solver": "DenseNewton})"_json;

    auto solver = polysolve::nonlinear::Solver::create(
        linear_solver_params,
        linear_solver_params,
        1,
        logger());

    auto x = problem.initial_value();
    try {
        solver->minimize(problem, x);
    } catch (const std::exception&) {
        return false;
    }
    return true;
}

} // namespace wmtk::operations
