#include "SeamlessSmoothing.hpp"

#include <polysolve/nonlinear/Problem.hpp>
#include <polysolve/nonlinear/Solver.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/MutableAccessor.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SeamlessConstraints.hpp>
namespace wmtk::operations {
class SeamlessSmoothing::SeamlessProblem : public polysolve::nonlinear::Problem
{
public:
    using typename polysolve::nonlinear::Problem::Scalar;
    using typename polysolve::nonlinear::Problem::THessian;
    using typename polysolve::nonlinear::Problem::TVector;

    SeamlessProblem(
        attribute::MutableAccessor<double>&& handle,
        const std::vector<simplex::Simplex>& simplices,
        const std::vector<Eigen::Matrix<double, 2, 2>>& rotation_matrix,
        const std::vector<int>& rotate_to,
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
    attribute::MutableAccessor<double> m_accessor;
    const std::vector<simplex::Simplex>& m_simplices;
    const std::vector<Eigen::Matrix<double, 2, 2>>& m_rotation_matrix;
    const std::vector<int>& m_rotate_to;
    Eigen::Vector2d x_init;
    const wmtk::function::Function& m_energy;
    invariants::InvariantCollection& m_invariants;
};

SeamlessSmoothing::SeamlessProblem::SeamlessProblem(
    attribute::MutableAccessor<double>&& accessor,
    const std::vector<simplex::Simplex>& simplices,
    const std::vector<Eigen::Matrix<double, 2, 2>>& rotation_matrix,
    const std::vector<int>& rotate_to,
    invariants::InvariantCollection& invariants,
    const wmtk::function::Function& energy)
    : m_accessor(std::move(accessor))
    , m_simplices(simplices)
    , m_rotation_matrix(rotation_matrix)
    , m_rotate_to(rotate_to)
    , m_energy(energy)
    , m_invariants(invariants)
{
    x_init = m_accessor.vector_attribute(m_simplices[0].tuple());
}

SeamlessSmoothing::SeamlessProblem::TVector SeamlessSmoothing::SeamlessProblem::initial_value()
    const
{
    return m_accessor.vector_attribute(m_simplices[0].tuple());
}

// ok
double SeamlessSmoothing::SeamlessProblem::value(const TVector& x)
{
    std::vector<TVector> tmps(m_simplices.size());
    double energy = 0.0;
    // backup
    for (int i = 0; i < m_simplices.size(); ++i) {
        tmps[i] = m_accessor.vector_attribute(m_simplices[i].tuple());
    }
    // update attribute
    solution_changed(x);
    // get local energy and sum it up
    for (int i = 0; i < m_simplices.size(); ++i) {
        energy += m_energy.get_value(m_simplices[i]);
    }
    // restore
    for (int i = 0; i < m_simplices.size(); ++i) {
        m_accessor.vector_attribute(m_simplices[i].tuple()) = tmps[i];
    }
    return energy;
}


// ok
void SeamlessSmoothing::SeamlessProblem::gradient(const TVector& x, TVector& gradv)
{
    std::vector<TVector> tmps(m_simplices.size());
    std::vector<TVector> gradvs(m_simplices.size());
    // backup
    for (int i = 0; i < m_simplices.size(); ++i) {
        tmps[i] = m_accessor.vector_attribute(m_simplices[i].tuple());
    }
    // update attribute
    solution_changed(x);
    // get local gradient and then restore
    for (int i = 0; i < m_simplices.size(); ++i) {
        gradvs[i] = m_energy.get_gradient(m_simplices[i]);
        m_accessor.vector_attribute(m_simplices[i].tuple()) = tmps[i];
    }
    // compute the overall gradient
    gradv = gradvs[0];
    Eigen::Matrix<double, 2, 2> current_rotation_matrix;
    current_rotation_matrix.setIdentity();
    for (int i = 0; i < m_simplices.size() - 1; ++i) {
        current_rotation_matrix = current_rotation_matrix * m_rotation_matrix[i].transpose();
        gradv += current_rotation_matrix * gradvs[m_rotate_to[i]];
    }
}

// ok
void SeamlessSmoothing::SeamlessProblem::hessian(const TVector& x, Eigen::MatrixXd& hessian)
{
    std::vector<TVector> tmps(m_simplices.size());
    std::vector<Eigen::MatrixXd> hessians(m_simplices.size());
    // backup
    for (int i = 0; i < m_simplices.size(); ++i) {
        tmps[i] = m_accessor.vector_attribute(m_simplices[i].tuple());
    }
    // update attribute
    solution_changed(x);
    // get local hessian and then restore
    for (int i = 0; i < m_simplices.size(); ++i) {
        hessians[i] = m_energy.get_hessian(m_simplices[i]);
        m_accessor.vector_attribute(m_simplices[i].tuple()) = tmps[i];
    }
    // compute the overall hessian
    hessian = hessians[0];
    Eigen::Matrix<double, 2, 2> current_rotation_matrix;
    current_rotation_matrix.setIdentity();
    for (int i = 0; i < m_simplices.size() - 1; ++i) {
        current_rotation_matrix = current_rotation_matrix * m_rotation_matrix[i].transpose();
        hessian += current_rotation_matrix * hessians[m_rotate_to[i]] *
                   current_rotation_matrix.transpose();
    }
}

// seems to be okay
void SeamlessSmoothing::SeamlessProblem::solution_changed(const TVector& new_x)
{
    TVector cur_dir = new_x - x_init;
    for (int i = 0; i < m_simplices.size(); ++i) {
        if (i == 0) {
            m_accessor.vector_attribute(m_simplices[0].tuple()) += cur_dir;
        } else {
            m_accessor.vector_attribute(m_simplices[m_rotate_to[i - 1]].tuple()) += cur_dir;
        }
        cur_dir = m_rotation_matrix[i] * cur_dir;
    }
}

bool SeamlessSmoothing::SeamlessProblem::is_step_valid(const TVector& x0, const TVector& x1)
{
    std::vector<TVector> tmps(m_simplices.size());
    // backup
    for (int i = 0; i < m_simplices.size(); ++i) {
        tmps[i] = m_accessor.vector_attribute(m_simplices[i].tuple());
    }
    // update attribute
    solution_changed(x1);


    bool res = true;
    for (int i = 0; i < m_simplices.size(); ++i) {
        auto domain = m_energy.domain(m_simplices[i]);
        std::vector<Tuple> dom_tmp;
        dom_tmp.reserve(domain.size());
        std::transform(
            domain.begin(),
            domain.end(),
            std::back_inserter(dom_tmp),
            [&](const simplex::Simplex& s) {
                return m_energy.mesh().map_tuples(m_invariants.mesh(), s).front();
            });

        res = res && m_invariants.after({}, dom_tmp);
        if (!res) {
            break;
        }
    }

    // restore
    for (int i = 0; i < m_simplices.size(); ++i) {
        m_accessor.vector_attribute(m_simplices[i].tuple()) = tmps[i];
    }
    return res;
}

// ok
SeamlessSmoothing::SeamlessSmoothing(
    TriMesh& ref_mesh,
    TriMesh& cut_mesh,
    std::shared_ptr<wmtk::function::Function> energy)
    : OptimizationSmoothing(ref_mesh, energy)
    , m_cut_mesh(cut_mesh)
    , m_ref_mesh(ref_mesh)
{}

std::vector<simplex::Simplex> SeamlessSmoothing::execute(const simplex::Simplex& simplex)
{
    // map simplex to cut_mesh
    std::vector<simplex::Simplex> vs_on_cut_mesh = mesh().map_to_child(m_cut_mesh, simplex);
    // if (m_cut_mesh.is_boundary(vs_on_cut_mesh[0])) {
    if (false) {
        // std::cout << "optimizing on boundary" << std::endl;
        // get all rotation matrices
        auto find_next_bd_edge = [this](const Tuple input_edge_tuple) -> Tuple {
            Tuple cur_edge = input_edge_tuple;
            cur_edge = this->m_cut_mesh.switch_edge(cur_edge);

            while (!this->m_cut_mesh.is_boundary(simplex::Simplex(PrimitiveType::Edge, cur_edge))) {
                cur_edge = this->m_cut_mesh.switch_face(cur_edge);
                cur_edge = this->m_cut_mesh.switch_edge(cur_edge);
            }
            return cur_edge;
        };

        std::vector<Eigen::Matrix<double, 2, 2>> rotation_matrix;
        std::vector<int> rotate_to;
        Tuple cur_edge_tuple = vs_on_cut_mesh.front().tuple();
        cur_edge_tuple = find_next_bd_edge(cur_edge_tuple);
        do {
            simplex::Simplex cur_edge = simplex::Simplex(PrimitiveType::Edge, cur_edge_tuple);
            simplex::Simplex pair_edge =
                wmtk::utils::get_pair_edge(m_ref_mesh, m_cut_mesh, cur_edge);
            Eigen::Matrix<double, 2, 2> rotation_matrix_i = wmtk::utils::get_rotation_matrix(
                m_cut_mesh,
                m_energy->attribute_handle().as<double>(),
                cur_edge,
                pair_edge);
            rotation_matrix.emplace_back(rotation_matrix_i);
            Tuple pair_edge_tuple = pair_edge.tuple();
            for (int i = 0; i < vs_on_cut_mesh.size(); ++i) {
                if (wmtk::simplex::utils::SimplexComparisons::equal(
                        m_cut_mesh,
                        simplex::Simplex(PrimitiveType::Vertex, pair_edge_tuple),
                        vs_on_cut_mesh[i])) {
                    rotate_to.emplace_back(i);
                    break;
                }
            }
            if (rotate_to.back() == 0) {
                break;
            }
            cur_edge_tuple = find_next_bd_edge(pair_edge_tuple);
        } while (true);
        // check if the vertex is a singular vertex
        {
            Eigen::Matrix<double, 2, 2> test_matrix = rotation_matrix[0];
            for (int i = 1; i < rotation_matrix.size(); ++i) {
                test_matrix = test_matrix * rotation_matrix[i];
            }
            if (!test_matrix.isIdentity()) {
                // wmtk::logger().info("singular vertex detected");
                return {};
            }
        }
        //  create problem and use solver to solve its
        auto accessor = m_energy->mesh().create_accessor(m_energy->attribute_handle().as<double>());
        SeamlessProblem problem(
            std::move(accessor),
            vs_on_cut_mesh,
            rotation_matrix,
            rotate_to,
            m_invariants,
            *m_energy);
        try {
            auto x = problem.initial_value();
            m_solver->minimize(problem, x);
        } catch (const std::exception&) {
            return {};
        }
    } else {
        return OptimizationSmoothing::execute(
            simplex); // as the new  execute on OptimizationSmoothing is supporting mapping
    }
    return AttributesUpdate::execute(simplex);
}

} // namespace wmtk::operations
