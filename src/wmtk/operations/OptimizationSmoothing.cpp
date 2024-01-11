#include "OptimizationSmoothing.hpp"

#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/MutableAccessor.hpp>
#include <wmtk/utils/Logger.hpp>

#include <polysolve/nonlinear/Solver.hpp>

namespace wmtk::operations {

class OptimizationSmoothing::WMTKProblem : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    WMTKProblem(
        attribute::MutableAccessor<double>&& handle,
        const simplex::Simplex& simplex,
        InvariantCollection& invariants,
        const wmtk::function::Function& energy);

    TVector initial_value() const;

    double value(const TVector& x) override;
    void gradient(const TVector& x, TVector& gradv) override;
    void hessian(const TVector& x, THessian& hessian) override
    {
        throw std::runtime_error("Sparse functions do not exist, use dense solver");
    }
    void hessian(const TVector& x, Eigen::MatrixXd& hessian) override;

    void solution_changed(const TVector& new_x) override;

    bool is_step_valid(const TVector& x0, const TVector& x1) override;

private:
    attribute::MutableAccessor<double> m_accessor;
    const simplex::Simplex& m_simplex;
    const wmtk::function::Function& m_energy;

    InvariantCollection& m_invariants;
};

OptimizationSmoothing::WMTKProblem::WMTKProblem(
    attribute::MutableAccessor<double>&& accessor,
    const simplex::Simplex& simplex,
    InvariantCollection& invariants,
    const wmtk::function::Function& energy)
    : m_accessor(std::move(accessor))
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

    auto domain = m_energy.domain(m_simplex);
    std::vector<Tuple> dom_tmp;
    dom_tmp.reserve(domain.size());

    if (&m_energy.mesh() == &m_invariants.mesh()) {
        std::transform(
            domain.begin(),
            domain.end(),
            std::back_inserter(dom_tmp),
            [](const simplex::Simplex& s) { return s.tuple(); });
    } else {
        std::transform(
            domain.begin(),
            domain.end(),
            std::back_inserter(dom_tmp),
            [&](const simplex::Simplex& s) {
                return m_energy.mesh().map_tuples(m_invariants.mesh(), s).front();
            });
    }
    // need to map the domain to the invariant mesh

    bool res = m_invariants.after({}, dom_tmp);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;

    return res;
}


OptimizationSmoothing::OptimizationSmoothing(
    Mesh& m,
    std::shared_ptr<wmtk::function::Function> energy)
    : AttributesUpdate(m)
    , m_energy(energy)
{
    m_linear_solver_params = R"({"solver": "Eigen::LDLT"})"_json;
    m_nonlinear_solver_params = R"({"solver": "DenseNewton"})"_json;

    create_solver();
}

void OptimizationSmoothing::create_solver()
{
    opt_logger().set_level(spdlog::level::off);
    m_solver = polysolve::nonlinear::Solver::create(
        m_nonlinear_solver_params,
        m_linear_solver_params,
        1,
        opt_logger());
}


std::vector<simplex::Simplex> OptimizationSmoothing::execute(const simplex::Simplex& simplex)
{
    std::vector<simplex::Simplex> simplices_to_smooth;

    if (&mesh() == &m_energy->mesh()) {
        simplices_to_smooth.push_back(simplex);
    } else {
        simplices_to_smooth = mesh().map(m_energy->mesh(), simplex);
    }

    for (auto s : simplices_to_smooth) {
        WMTKProblem problem(
            m_energy->mesh().create_accessor(m_energy->attribute_handle().as<double>()),
            s,
            m_invariants,
            *m_energy);

        auto x = problem.initial_value();
        try {
            m_solver->minimize(problem, x);
        } catch (const std::exception&) {
            return {};
        }
    }


    return AttributesUpdate::execute(simplex);
}

} // namespace wmtk::operations
