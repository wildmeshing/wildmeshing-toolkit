#include "SeamlessSmoothing.hpp"

#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/MutableAccessor.hpp>
#include <wmtk/utils/Logger.hpp>

#include <polysolve/nonlinear/Solver.hpp>

namespace wmtk::operations {

class SeamlessSmoothing::WMTKProblem : public polysolve::nonlinear::Problem
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

SeamlessSmoothing::WMTKProblem::WMTKProblem(
    attribute::MutableAccessor<double>&& accessor,
    const simplex::Simplex& simplex,
    InvariantCollection& invariants,
    const wmtk::function::Function& energy)
    : m_accessor(std::move(accessor))
    , m_simplex(simplex)
    , m_energy(energy)
    , m_invariants(invariants)
{}

SeamlessSmoothing::WMTKProblem::TVector SeamlessSmoothing::WMTKProblem::initial_value() const
{
    return m_accessor.vector_attribute(m_simplex.tuple());
}

double SeamlessSmoothing::WMTKProblem::value(const TVector& x)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    double res = m_energy.get_value(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;

    return res;
}

void SeamlessSmoothing::WMTKProblem::gradient(const TVector& x, TVector& gradv)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    gradv = m_energy.get_gradient(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;
}

void SeamlessSmoothing::WMTKProblem::hessian(const TVector& x, Eigen::MatrixXd& hessian)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x;
    hessian = m_energy.get_hessian(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;
}

void SeamlessSmoothing::WMTKProblem::solution_changed(const TVector& new_x)
{
    m_accessor.vector_attribute(m_simplex.tuple()) = new_x;
}


bool SeamlessSmoothing::WMTKProblem::is_step_valid(const TVector& x0, const TVector& x1)
{
    TVector tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = x1;

    auto domain = m_energy.domain(m_simplex);
    std::vector<Tuple> dom_tmp;
    dom_tmp.reserve(domain.size());
    std::transform(
        domain.begin(),
        domain.end(),
        std::back_inserter(dom_tmp),
        [](const simplex::Simplex& s) { return s.tuple(); });

    bool res = m_invariants.after({}, dom_tmp);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;

    return res;
}


SeamlessSmoothing::SeamlessSmoothing(std::shared_ptr<wmtk::function::Function> energy)
    : AttributesUpdate(energy->mesh())
    , m_energy(energy)
{
    m_linear_solver_params = R"({"solver": "Eigen::LDLT"})"_json;
    m_nonlinear_solver_params = R"({"solver": "DenseNewton"})"_json;

    create_solver();
}

void SeamlessSmoothing::create_solver()
{
    m_solver = polysolve::nonlinear::Solver::create(
        m_nonlinear_solver_params,
        m_linear_solver_params,
        1,
        opt_logger());
}


std::vector<simplex::Simplex> SeamlessSmoothing::execute(const simplex::Simplex& simplex)
{
    WMTKProblem problem(
        mesh().create_accessor(m_energy->attribute_handle().as<double>()),
        simplex,
        m_invariants,
        *m_energy);

    auto x = problem.initial_value();
    try {
        m_solver->minimize(problem, x);
    } catch (const std::exception&) {
        return {};
    }


    return AttributesUpdate::execute(simplex);
}

} // namespace wmtk::operations
