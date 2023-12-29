#include "OptimizationSmoothing.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <polysolve/nonlinear/Solver.hpp>

namespace wmtk::operations {

OptimizationSmoothing::WMTKProblem::WMTKProblem(
    Mesh& mesh,
    const MeshAttributeHandle<double>& handle,
    const simplex::Simplex& simplex,
    InvariantCollection& invariants,
    const wmtk::function::Function& energy)
    : m_handle(handle)
    , m_accessor(mesh.create_accessor(handle))
    , m_simplex(simplex)
    , m_energy(energy)
    , m_invariants(invariants)
{}

OptimizationSmoothing::WMTKProblem::TVector OptimizationSmoothing::WMTKProblem::initial_value()
    const
{
    return m_accessor.vector_attribute(m_simplex.tuple());
}

double OptimizationSmoothing::WMTKProblem::value(const TVector& x)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    double res = m_energy.get_value(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;

    return res;
}

void OptimizationSmoothing::WMTKProblem::gradient(const TVector& x, TVector& gradv)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    gradv = m_energy.get_gradient(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;
}

void OptimizationSmoothing::WMTKProblem::hessian(const TVector& x, Eigen::MatrixXd& hessian)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    hessian = m_energy.get_hessian(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;
}

void OptimizationSmoothing::WMTKProblem::solution_changed(const TVector& new_x)
{
    m_accessor.vector_attribute(m_simplex.tuple()) = new_x;
}


bool OptimizationSmoothing::WMTKProblem::is_step_valid(const TVector& x0, const TVector& x1)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x1;

    bool res = m_invariants.before(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;

    return res;
}


OptimizationSmoothing::OptimizationSmoothing(std::shared_ptr<wmtk::function::Function> energy)
    : AttributesUpdateBase(energy->mesh())
    , m_energy(energy)
{
    m_linear_solver_params = R"({"solver": "Eigen::LDLT"})"_json;
    m_nonlinear_solver_params = R"({"solver": "DenseNewton"})"_json;

    create_solver();
}

void OptimizationSmoothing::create_solver()
{
    m_solver = polysolve::nonlinear::Solver::create(
        m_nonlinear_solver_params,
        m_linear_solver_params,
        1,
        opt_logger());
}


std::vector<Simplex> OptimizationSmoothing::execute(const Simplex& simplex)
{
    WMTKProblem problem(mesh(), m_energy->attribute_handle(), simplex, m_invariants, *m_energy);

    auto x = problem.initial_value();
    try {
        m_solver->minimize(problem, x);
    } catch (const std::exception&) {
        return {};
    }


    return AttributesUpdateBase::execute(simplex);
}

} // namespace wmtk::operations
