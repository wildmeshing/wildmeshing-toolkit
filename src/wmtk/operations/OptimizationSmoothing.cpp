#include "OptimizationSmoothing.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/Mesh.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/Logger.hpp>

#include <polysolve/nonlinear/Solver.hpp>

namespace wmtk::operations {

OptimizationSmoothing::WMTKProblem::WMTKProblem(
    Mesh& mesh,
    const MeshAttributeHandle<double>& handle,
    const simplex::Simplex& simplex,
    const std::unique_ptr<wmtk::function::Function>& energy)
    : m_handle(handle)
    , m_accessor(mesh.create_accessor(handle))
    , m_simplex(simplex)
    , m_energy(energy)
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
    double res = m_energy->get_value(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;

    return res;
}

void OptimizationSmoothing::WMTKProblem::gradient(const TVector& x, TVector& gradv)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    gradv = m_energy->get_gradient(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;
}

void OptimizationSmoothing::WMTKProblem::hessian(const TVector& x, Eigen::MatrixXd& hessian)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    hessian = m_energy->get_hessian(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;
}

void OptimizationSmoothing::WMTKProblem::solution_changed(const TVector& new_x)
{
    m_accessor.vector_attribute(m_simplex.tuple()) = new_x;
}


bool OptimizationSmoothing::WMTKProblem::is_step_valid(const TVector& x0, const TVector& x1) const
{
    // TODO use invariants
    return true;
}

void OperationSettings<OptimizationSmoothing>::create_invariants()
{
    OperationSettings<AttributesUpdateBase>::create_invariants();
    // if (!smooth_boundary) {
    //     invariants->add(std::make_unique<InteriorVertexInvariant>(m_mesh));
    // }
    if (m_mesh.top_simplex_type() == PrimitiveType::Face)
        invariants->add(std::make_shared<TriangleInversionInvariant>(m_mesh, coordinate_handle));
    // else if (m_mesh.top_simplex_type() == PrimitiveType::Tetrahedron)
    //     invariants->add(std::make_shared<TetrahedronInversionInvariant>(m_mesh,
    //     coordinate_handle));
}

OptimizationSmoothing::OptimizationSmoothing(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<OptimizationSmoothing>& settings)
    : AttributesUpdateBase(m, t, settings)
    , m_settings{settings}
{}

std::string OptimizationSmoothing::name() const
{
    return "opt_smoothing";
}

Accessor<double> OptimizationSmoothing::coordinate_accessor()
{
    return mesh().create_accessor(m_settings.coordinate_handle);
}
ConstAccessor<double> OptimizationSmoothing::const_coordinate_accessor() const
{
    return mesh().create_const_accessor(m_settings.coordinate_handle);
}

bool OptimizationSmoothing::execute()
{
    WMTKProblem problem(mesh(), coordinate_handle(), input_simplex(), m_settings.energy);

    polysolve::json linear_solver_params = R"({"solver": "Eigen::LDLT"})"_json;
    polysolve::json nonlinear_solver_params = R"({"solver": "DenseNewton"})"_json;

    auto solver = polysolve::nonlinear::Solver::create(
        nonlinear_solver_params,
        linear_solver_params,
        1,
        logger());

    auto x = problem.initial_value();
    try {
        solver->minimize(problem, x);
    } catch (const std::exception&) {
        return false;
    }


    return AttributesUpdateBase::execute();
}

} // namespace wmtk::operations
