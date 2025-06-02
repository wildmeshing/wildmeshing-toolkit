#include "AMIPSOptimizationSmoothing.hpp"

#include <polysolve/nonlinear/Problem.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/cofaces_single_dimension_iterable.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
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
// Enables the use of power of p to approximate minimizing max energy
// #define WMTK_ENABLE_P
#ifdef WMTK_ENABLE_P
    const int p = 4;
#endif
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
#ifdef WMTK_ENABLE_P
            res += std::pow(wmtk::function::Tri_AMIPS_energy(c), p);
#else
            res += wmtk::function::Tri_AMIPS_energy(c);
#endif
        } else {
            assert(x.size() == 3);
            c[0] = x[0];
            c[1] = x[1];
            c[2] = x[2];
#ifdef WMTK_ENABLE_P
            res += std::pow(wmtk::function::Tet_AMIPS_energy(c), p);
#else
            res += wmtk::function::Tet_AMIPS_energy(c);
#endif
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
#ifdef WMTK_ENABLE_P
    double tmp2 = 0;
#endif

    for (auto c : m_cells) {
        if constexpr (S == 6) {
            assert(x.size() == 2);
            c[0] = x[0];
            c[1] = x[1];
            wmtk::function::Tri_AMIPS_jacobian(c, tmp);
#ifdef WMTK_ENABLE_P
            tmp2 = wmtk::function::Tri_AMIPS_energy(c);
#endif
        } else {
            assert(x.size() == 3);
            c[0] = x[0];
            c[1] = x[1];
            c[2] = x[2];
            wmtk::function::Tet_AMIPS_jacobian(c, tmp);
#ifdef WMTK_ENABLE_P
            tmp2 = wmtk::function::Tet_AMIPS_energy(c);
#endif
        }
#ifdef WMTK_ENABLE_P
        gradv += double(p) * pow(tmp2, p - 1) * tmp;
#else
        gradv += tmp;
#endif
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
#ifdef WMTK_ENABLE_P
    double tmp2 = 0;
    Eigen::Matrix<double, size, 1> tmpj(size);
#endif

    for (auto c : m_cells) {
        if constexpr (S == 6) {
            assert(x.size() == 2);
            c[0] = x[0];
            c[1] = x[1];
            wmtk::function::Tri_AMIPS_hessian(c, tmp);
#ifdef WMTK_ENABLE_P
            wmtk::function::Tri_AMIPS_jacobian(c, tmpj);
            tmp2 = wmtk::function::Tri_AMIPS_energy(c);
#endif
        } else {
            assert(x.size() == 3);
            c[0] = x[0];
            c[1] = x[1];
            c[2] = x[2];
            wmtk::function::Tet_AMIPS_hessian(c, tmp);
#ifdef WMTK_ENABLE_P
            wmtk::function::Tet_AMIPS_jacobian(c, tmpj);
            tmp2 = wmtk::function::Tet_AMIPS_energy(c);
#endif
        }
#ifdef WMTK_ENABLE_P
        hessian += (p) * (p - 1) * std::pow(tmp2, p - 2) * tmpj * tmpj.transpose() +
                   p * std::pow(tmp2, p - 1) * tmp;
#else
        hessian += tmp;
#endif
        // 12 f(x)^2 * f(x)'*f^T'(x) + 4 f(x)^3 * H
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

            Eigen::Matrix<double, size, 1> x0m = x0;
            assert(wmtk::utils::wmtk_orient3d(p3, x0m, p1, p2) > 0);
            if (wmtk::utils::wmtk_orient3d(p3, p0, p1, p2) <= 0) {
                return false;
            }
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
    // assert(m_coordinate_handle.holds<Rational>());

    m_linear_solver_params = R"({"solver": "Eigen::LDLT"})"_json;
    m_nonlinear_solver_params = R"({"solver": "DenseNewton", "max_iterations": 10})"_json;
    // m_nonlinear_solver_params = R"({"solver": "L-BFGS", "max_iterations": 100,
    // "advanced":{"apply_gradient_fd": "FullFiniteDiff"}})"_json;
    //  m_nonlinear_solver_params = R"({"solver": "GradientDescent", "max_iterations": 100})"_json;


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
    if (m_coordinate_handle.holds<Rational>()) {
        auto accessor = mesh().create_accessor(m_coordinate_handle.as<Rational>());

        auto neighs = wmtk::simplex::cofaces_single_dimension_iterable(
            mesh(),
            simplex,
            mesh().top_simplex_type());

        if (mesh().top_simplex_type() == PrimitiveType::Triangle) {
            std::vector<std::array<double, 6>> cells;

            for (const Tuple& cell_tuple : neighs) {
                const simplex::Simplex cell(mesh().top_simplex_type(), cell_tuple);
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

            for (const Tuple& cell_tuple : neighs) {
                simplex::Simplex cell(mesh().top_simplex_type(), cell_tuple);

                if (!mesh().is_ccw(cell.tuple())) {
                    // switch any local id but NOT the vertex
                    cell = simplex::Simplex(
                        mesh(),
                        cell.primitive_type(),
                        mesh().switch_tuple(cell.tuple(), PrimitiveType::Edge));
                }
                assert(mesh().is_ccw(cell.tuple()));

                const auto vertices =
                    simplex::faces_single_dimension(mesh(), cell, PrimitiveType::Vertex);


                assert(vertices.size() == 4);
                if (!simplex::utils::SimplexComparisons::equal(
                        mesh(),
                        vertices.simplex_vector()[0],
                        simplex)) {
                    std::cout << "error here" << std::endl;
                }


                std::array<double, 12> single_cell;
                std::vector<Vector3r> ps;
                for (size_t i = 0; i < 4; ++i) {
                    const simplex::Simplex& v = vertices.simplex_vector()[i];
                    // const auto p = accessor.const_vector_attribute(vertices[i]);
                    const auto p = accessor.const_vector_attribute(v);
                    ps.push_back(p);
                    single_cell[3 * i + 0] = p[0].to_double();
                    single_cell[3 * i + 1] = p[1].to_double();
                    single_cell[3 * i + 2] = p[2].to_double();

                    if (!p[0].is_rounded() || !p[1].is_rounded() || !p[2].is_rounded()) return {};
                }
                cells.emplace_back(single_cell);

                // cells.emplace_back(m_amips.get_raw_coordinates<4, 3>(cell, simplex));
            }
            WMTKAMIPSProblem<12> problem(cells);

            {
                Eigen::Vector3d x1(cells[0][0], cells[0][1], cells[0][2]);
                if (!problem.is_step_valid(x1, x1)) {
                    std::cout << "step is not valid!!!!!!!!!!!!!!!!" << std::endl;
                }
            }

            auto x = problem.initial_value();
            auto x0 = problem.initial_value();

            try {
                m_solver->minimize(problem, x);

            } catch (const std::exception&) {
            }

            // Hack for surface only


            double alpha = 1.00;

            for (int64_t d = 0; d < m_coordinate_handle.dimension(); ++d) {
                // accessor.vector_attribute(simplex.tuple())[d] = Rational(x[d], true);
                accessor.vector_attribute(simplex.tuple())[d] =
                    Rational((1 - alpha) * x0[d] + alpha * x[d], true);
            }
        }

        // assert(attribute_handle() == m_function.attribute_handle());

        double res = 0;
    } else {
        auto accessor = mesh().create_accessor(m_coordinate_handle.as<double>());

        auto neighs = wmtk::simplex::cofaces_single_dimension_iterable(
            mesh(),
            simplex,
            mesh().top_simplex_type());

        if (mesh().top_simplex_type() == PrimitiveType::Triangle) {
            std::vector<std::array<double, 6>> cells;

            for (const Tuple& cell_tuple : neighs) {
                const simplex::Simplex cell(mesh().top_simplex_type(), cell_tuple);
                cells.emplace_back(m_amips.get_raw_coordinates<3, 2>(cell, simplex));
            }
            WMTKAMIPSProblem<6> problem(cells);

            auto x = problem.initial_value();
            try {
                m_solver->minimize(problem, x);

                for (int64_t d = 0; d < m_coordinate_handle.dimension(); ++d) {
                    accessor.vector_attribute(simplex.tuple())[d] = x[d];
                }
            } catch (const std::exception&) {
                return {};
            }
        } else {
            assert(mesh().top_simplex_type() == PrimitiveType::Tetrahedron);

            std::vector<std::array<double, 12>> cells;

            for (const Tuple& cell_tuple : neighs) {
                simplex::Simplex cell(mesh().top_simplex_type(), cell_tuple);

                if (!mesh().is_ccw(cell.tuple())) {
                    // switch any local id but NOT the vertex
                    cell = simplex::Simplex(
                        mesh(),
                        cell.primitive_type(),
                        mesh().switch_tuple(cell.tuple(), PrimitiveType::Edge));
                }
                assert(mesh().is_ccw(cell.tuple()));

                const auto vertices =
                    simplex::faces_single_dimension(mesh(), cell, PrimitiveType::Vertex);


                assert(vertices.size() == 4);
                if (!simplex::utils::SimplexComparisons::equal(
                        mesh(),
                        vertices.simplex_vector()[0],
                        simplex)) {
                    std::cout << "error here" << std::endl;
                }


                std::array<double, 12> single_cell;
                std::vector<Vector3d> ps;
                for (size_t i = 0; i < 4; ++i) {
                    const simplex::Simplex& v = vertices.simplex_vector()[i];
                    // const auto p = accessor.const_vector_attribute(vertices[i]);
                    const auto p = accessor.const_vector_attribute(v);
                    ps.push_back(p);
                    single_cell[3 * i + 0] = p[0];
                    single_cell[3 * i + 1] = p[1];
                    single_cell[3 * i + 2] = p[2];
                }

                cells.emplace_back(single_cell);

                // cells.emplace_back(m_amips.get_raw_coordinates<4, 3>(cell, simplex));
            }
            WMTKAMIPSProblem<12> problem(cells);

            {
                Eigen::Vector3d x1(cells[0][0], cells[0][1], cells[0][2]);
                if (!problem.is_step_valid(x1, x1)) {
                    std::cout << "step is not valid!!!!!!!!!!!!!!!!" << std::endl;
                }
            }

            auto x = problem.initial_value();
            auto x0 = problem.initial_value();

            try {
                m_solver->minimize(problem, x);

            } catch (const std::exception&) {
            }

            // Hack for surface only

            double alpha = 1.00;

            for (int64_t d = 0; d < m_coordinate_handle.dimension(); ++d) {
                // accessor.vector_attribute(simplex.tuple())[d] = Rational(x[d], true);
                accessor.vector_attribute(simplex.tuple())[d] = (1 - alpha) * x0[d] + alpha * x[d];
            }
        }

        // assert(attribute_handle() == m_function.attribute_handle());

        double res = 0;
    }


    return AttributesUpdate::execute(simplex);
}

} // namespace wmtk::operations
