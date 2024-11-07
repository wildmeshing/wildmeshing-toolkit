#include "OptimizationSmoothing.hpp"

#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/utils/Logger.hpp>

#include <polysolve/nonlinear/Solver.hpp>

#include <Eigen/Core>

namespace wmtk::operations {

namespace {
template <typename T>
const Eigen::Matrix<T, -1, 1> convert(const polysolve::nonlinear::Problem::TVector& v)
{
    return v;
}

template <>
const Eigen::Matrix<Rational, -1, 1> convert(const polysolve::nonlinear::Problem::TVector& v)
{
    Eigen::Matrix<Rational, -1, 1> res(v.size());
    for (int64_t d = 0; d < v.size(); ++d) {
        res[d] = Rational(v[d], true);
    }
    return res;
}
} // namespace


template <typename T>
class OptimizationSmoothing::WMTKProblem : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    WMTKProblem(
        attribute::Accessor<T>&& handle,
        const simplex::Simplex& simplex,
        invariants::InvariantCollection& invariants,
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
    attribute::Accessor<T> m_accessor;
    const simplex::Simplex& m_simplex;
    const wmtk::function::Function& m_energy;

    invariants::InvariantCollection& m_invariants;
};

template <typename T>
OptimizationSmoothing::WMTKProblem<T>::WMTKProblem(
    attribute::Accessor<T>&& accessor,
    const simplex::Simplex& simplex,
    invariants::InvariantCollection& invariants,
    const wmtk::function::Function& energy)
    : m_accessor(std::move(accessor))
    , m_simplex(simplex)
    , m_energy(energy)
    , m_invariants(invariants)
{}

template <typename T>
Eigen::Matrix<double, -1, 1> OptimizationSmoothing::WMTKProblem<T>::initial_value() const
{
    if constexpr (std::is_same_v<T, Rational>) {
        const Eigen::Matrix<T, -1, 1> tmp = m_accessor.const_vector_attribute(m_simplex.tuple());
        Eigen::Matrix<double, -1, 1> tmp1(tmp.size());
        for (int64_t d = 0; d < tmp1.size(); ++d) {
            tmp1[d] = tmp[d].to_double();
        }
        return tmp1;
    } else {
        return m_accessor.const_vector_attribute(m_simplex.tuple());
    }
}

template <typename T>
double OptimizationSmoothing::WMTKProblem<T>::value(const TVector& x)
{
    auto tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = convert<T>(x);
    double res = m_energy.get_value(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;

    return res;
}

template <typename T>
void OptimizationSmoothing::WMTKProblem<T>::gradient(const TVector& x, TVector& gradv)
{
    auto tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = convert<T>(x);
    gradv = m_energy.get_gradient(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;
}

template <typename T>
void OptimizationSmoothing::WMTKProblem<T>::hessian(const TVector& x, Eigen::MatrixXd& hessian)
{
    auto tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = convert<T>(x);
    hessian = m_energy.get_hessian(m_simplex);

    m_accessor.vector_attribute(m_simplex.tuple()) = tmp;
}

template <typename T>
void OptimizationSmoothing::WMTKProblem<T>::solution_changed(const TVector& new_x)
{
    // m_accessor.vector_attribute(m_simplex.tuple()) = new_x;
}

template <typename T>
bool OptimizationSmoothing::WMTKProblem<T>::is_step_valid(const TVector& x0, const TVector& x1)
{
    auto tmp = m_accessor.vector_attribute(m_simplex.tuple());
    m_accessor.vector_attribute(m_simplex.tuple()) = convert<T>(x1);

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


OptimizationSmoothing::OptimizationSmoothing(std::shared_ptr<wmtk::function::Function> energy)
    : AttributesUpdate(energy->mesh())
    , m_energy(energy)
{
    operation_name = "OptimizationSmoothing";
    m_linear_solver_params = R"({"solver": "Eigen::LDLT"})"_json;
    m_nonlinear_solver_params = R"({"solver": "DenseNewton", "max_iterations": 10})"_json;

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


std::vector<simplex::Simplex> OptimizationSmoothing::execute(const simplex::Simplex& simplex)
{
    if (m_energy->attribute_handle().holds<double>()) {
        auto accessor = mesh().create_accessor(m_energy->attribute_handle().as<double>());
        WMTKProblem problem(std::move(accessor), simplex, m_invariants, *m_energy);

        auto x = problem.initial_value();
        try {
            m_solver->minimize(problem, x);

            accessor.vector_attribute(simplex.tuple()) = x;

        } catch (const std::exception&) {
            return {};
        }


        return AttributesUpdate::execute(simplex);
    } else {
        assert(m_energy->attribute_handle().holds<Rational>());
        auto accessor = mesh().create_accessor(m_energy->attribute_handle().as<Rational>());
        WMTKProblem problem(std::move(accessor), simplex, m_invariants, *m_energy);

        auto x = problem.initial_value();
        try {
            m_solver->minimize(problem, x);

            for (int64_t d = 0; d < m_energy->attribute_handle().dimension(); ++d) {
                accessor.vector_attribute(simplex.tuple())[d] = Rational(x[d], true);
            }
        } catch (const std::exception&) {
            return {};
        }


        return AttributesUpdate::execute(simplex);
    }
}

} // namespace wmtk::operations
