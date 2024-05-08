#include "AMIPSOptimizationSmoothing.hpp"

#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/orient.hpp>

#include <polysolve/nonlinear/Solver.hpp>

#include <Eigen/Core>

namespace wmtk::operations {

template <int S>
class AMIPSOptimizationSmoothing::WMTKAMIPSProblem : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    WMTKAMIPSProblem(std::vector<std::array<double, S>>& cells);

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
    std::vector<std::array<double, S>> m_cells;
};

template <int S>
AMIPSOptimizationSmoothing::WMTKAMIPSProblem<S>::WMTKAMIPSProblem(
    std::vector<std::array<double, S>>& cells)
    : m_cells(cells)
{}

template <int S>
Eigen::Matrix<double, -1, 1> AMIPSOptimizationSmoothing::WMTKAMIPSProblem<S>::initial_value() const
{
    constexpr int64_t size = S == 6 ? 2 : 3;
    Eigen::Matrix<double, size, 1> tmp;

    for (int64_t d = 0; d < size; ++d) {
        tmp(d) = m_cells[0][d];
    }

    return tmp;
}

template <int S>
double AMIPSOptimizationSmoothing::WMTKAMIPSProblem<S>::value(const TVector& x)
{
    double res = 0;
    for (auto c : m_cells) {
        if constexpr (S == 6) {
            assert(x.size() == 2);
            c[0] = x[0];
            c[1] = x[1];

            res += wmtk::function::Tri_AMIPS_energy(c);
        } else {
            assert(x.size() == 3);
            c[0] = x[0];
            c[1] = x[1];
            c[2] = x[2];
            res += wmtk::function::Tet_AMIPS_energy(c);
        }
    }

    return res;
}

template <int S>
void AMIPSOptimizationSmoothing::WMTKAMIPSProblem<S>::gradient(const TVector& x, TVector& gradv)
{
    constexpr int64_t size = S == 6 ? 2 : 3;
    gradv.resize(size);
    gradv.setZero();
    Eigen::Matrix<double, size, 1> tmp(size);


    for (auto c : m_cells) {
        if constexpr (S == 6) {
            assert(x.size() == 2);
            c[0] = x[0];
            c[1] = x[1];
            wmtk::function::Tri_AMIPS_jacobian(c, tmp);
        } else {
            assert(x.size() == 3);
            c[0] = x[0];
            c[1] = x[1];
            c[2] = x[2];
            wmtk::function::Tet_AMIPS_jacobian(c, tmp);
        }
        gradv += tmp;
    }
}

template <int S>
void AMIPSOptimizationSmoothing::WMTKAMIPSProblem<S>::hessian(
    const TVector& x,
    Eigen::MatrixXd& hessian)
{
    constexpr int64_t size = S == 6 ? 2 : 3;
    hessian.resize(size, size);
    hessian.setZero();
    Eigen::Matrix<double, size, size> tmp;


    for (auto c : m_cells) {
        if constexpr (S == 6) {
            assert(x.size() == 2);
            c[0] = x[0];
            c[1] = x[1];
            wmtk::function::Tri_AMIPS_hessian(c, tmp);
        } else {
            assert(x.size() == 3);
            c[0] = x[0];
            c[1] = x[1];
            c[2] = x[2];
            wmtk::function::Tet_AMIPS_hessian(c, tmp);
        }
        hessian += tmp;
    }
}

template <int S>
void AMIPSOptimizationSmoothing::WMTKAMIPSProblem<S>::solution_changed(const TVector& new_x)
{}

template <int S>
bool AMIPSOptimizationSmoothing::WMTKAMIPSProblem<S>::is_step_valid(
    const TVector& x0,
    const TVector& x1)
{
    constexpr int64_t size = S == 6 ? 2 : 3;
    Eigen::Matrix<double, size, 1> p0 = x1, p1, p2, p3;
    if constexpr (S == 6) {
        for (const auto& c : m_cells) {
            p1 << c[2], c[3];
            p2 << c[4], c[5];

            if (wmtk::utils::wmtk_orient2d(p0, p1, p2) <= 0) return false;
        }
    } else {
        for (const auto& c : m_cells) {
            p1 << c[3], c[4], c[5];
            p2 << c[6], c[7], c[8];
            p3 << c[9], c[10], c[11];

            if (wmtk::utils::wmtk_orient3d(p0, p1, p2, p3) <= 0) return false;
        }
    }
    return true;
}


AMIPSOptimizationSmoothing::AMIPSOptimizationSmoothing(
    Mesh& mesh,
    const attribute::MeshAttributeHandle& coords)
    : AttributesUpdate(mesh)
    , m_coordinate_handle(coords)
    , m_amips(mesh, coords)
{
    assert(m_coordinate_handle.holds<Rational>());

    m_linear_solver_params = R"({"solver": "Eigen::LDLT"})"_json;
    m_nonlinear_solver_params = R"({"solver": "DenseNewton", "max_iterations": 10})"_json;

    create_solver();
}

void AMIPSOptimizationSmoothing::create_solver()
{
    m_solver = polysolve::nonlinear::Solver::create(
        m_nonlinear_solver_params,
        m_linear_solver_params,
        1,
        opt_logger());
}


std::vector<simplex::Simplex> AMIPSOptimizationSmoothing::execute(const simplex::Simplex& simplex)
{
    auto accessor = mesh().create_accessor(m_coordinate_handle.as<Rational>());
    const auto neighs = wmtk::simplex::cofaces_single_dimension_simplices(
        mesh(),
        simplex,
        mesh().top_simplex_type());

    if (mesh().top_simplex_type() == PrimitiveType::Triangle) {
        std::vector<std::array<double, 6>> cells;

        for (const simplex::Simplex& cell : neighs) {
            cells.emplace_back(m_amips.get_raw_coordinates<3, 2>(cell, simplex));
        }
        WMTKAMIPSProblem<6> problem(cells);

        auto x = problem.initial_value();
        try {
            m_solver->minimize(problem, x);

            for (int64_t d = 0; d < m_coordinate_handle.dimension(); ++d) {
                accessor.vector_attribute(simplex.tuple())[d] = Rational(x[d], true);
            }
        } catch (const std::exception&) {
            return {};
        }
    } else {
        assert(mesh().top_simplex_type() == PrimitiveType::Tetrahedron);

        std::vector<std::array<double, 12>> cells;

        for (const simplex::Simplex& cell : neighs) {
            cells.emplace_back(m_amips.get_raw_coordinates<4, 3>(cell, simplex));
        }
        WMTKAMIPSProblem<12> problem(cells);

        auto x = problem.initial_value();
        try {
            m_solver->minimize(problem, x);

            for (int64_t d = 0; d < m_coordinate_handle.dimension(); ++d) {
                accessor.vector_attribute(simplex.tuple())[d] = Rational(x[d], true);
            }
        } catch (const std::exception&) {
            return {};
        }
    }

    // assert(attribute_handle() == m_function.attribute_handle());

    double res = 0;


    return AttributesUpdate::execute(simplex);
}

} // namespace wmtk::operations
