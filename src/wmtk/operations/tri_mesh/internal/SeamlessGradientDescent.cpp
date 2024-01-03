#include "SeamlessGradientDescent.hpp"
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SeamlessConstraints.hpp>
namespace wmtk::operations::tri_mesh::internal {
SeamlessGradientDescent::SeamlessGradientDescent(
    TriMesh& seamed_mesh,
    std::shared_ptr<TriMesh> cut_mesh,
    MeshAttributeHandle<double> uv_handle,
    const Simplex& v_on_seamed_mesh,
    const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings)
    : VertexSmoothUsingDifferentiableEnergy(seamed_mesh, v_on_seamed_mesh, settings)
    , m_cut_mesh{cut_mesh}
    , m_uv_handle{uv_handle}
{
    assert(v_on_seamed_mesh.primitive_type() == PrimitiveType::Vertex);
}
std::string SeamlessGradientDescent::name() const
{
    return "seamless_gradient_descent";
}


Eigen::VectorXd SeamlessGradientDescent::get_descent_direction(
    function::utils::DifferentiableFunctionEvaluator& f) const
{
    std::runtime_error("should not be touched");
    return -f.get_gradient();
}

double SeamlessGradientDescent::evaluate_energy() const
{
    return 0;
}


bool SeamlessGradientDescent::execute_on_interior(const Simplex& v_on_cut_mesh)
{
    auto pos_accessor = coordinate_accessor();
    auto uv_accessor = m_cut_mesh->create_accessor(m_uv_handle);


    auto cut_mesh_inversion_check = TriangleInversionInvariant(*m_cut_mesh, m_uv_handle);
    // get all triangles that need to check inversion
    std::vector<Simplex> triangle_simplex_to_check =
        wmtk::simplex::top_dimension_cofaces(*m_cut_mesh, v_on_cut_mesh).simplex_vector();
    std::vector<Tuple> triangle_tuples_to_check;
    for (const auto& s : triangle_simplex_to_check) {
        triangle_tuples_to_check.emplace_back(s.tuple());
    }

    // get the gradient descent direction
    Eigen::VectorXd dir = -m_settings.energy->get_gradient(v_on_cut_mesh);

    // line search here!!!
    double step_size = 1.0; // TODO: set as a parameter
    int max_steps = 50; //  TODO: set as a parameter
    int steps = 0;
    double energy_before = m_settings.energy->get_value(v_on_cut_mesh);
    double current_energy = energy_before;
    Eigen::Vector3d current_uv = uv_accessor.vector_attribute(v_on_cut_mesh.tuple());
    Eigen::Vector3d new_uv;

    do {
        new_uv = current_uv + step_size * dir;
        uv_accessor.vector_attribute(v_on_cut_mesh.tuple()) = new_uv;
        current_energy = m_settings.energy->get_value(v_on_cut_mesh);

        // wmtk::logger().info("energy before: {}", energy_before);
        // wmtk::logger().info("step_size: {}", step_size);
        // wmtk::logger().info("energy after: {}", current_energy);

        if (!std::isnan(current_energy) && !std::isinf(current_energy) &&
            current_energy < energy_before) {
            if (cut_mesh_inversion_check.after(PrimitiveType::Face, triangle_tuples_to_check)) {
                // wmtk::logger().info("opt success");
                return true;
            } else {
                // wmtk::logger().info("inversion detected");
            }
        }

        step_size /= 2;
    } while (steps++ < max_steps);
    // wmtk::logger().info("opt failed");
    return false;
}

bool SeamlessGradientDescent::execute_on_boundary(const std::vector<Simplex>& vs_on_cut_mesh)
{
    auto pos_accessor = coordinate_accessor();
    auto uv_accessor = m_cut_mesh->create_accessor(m_uv_handle);

    /////////////////////////////////////////////////////////////////////////////////
    // prepare for the triangle inversion check
    // get all the triangles that need to check inversion
    /////////////////////////////////////////////////////////////////////////////////
    auto cut_mesh_inversion_check = TriangleInversionInvariant(*m_cut_mesh, m_uv_handle);
    std::vector<Tuple> triangle_tuples_to_check;
    for (const auto& v_on_cut_mesh : vs_on_cut_mesh) {
        // get all triangles that need to check inversion
        const std::vector<Simplex> triangle_simplex_to_check =
            wmtk::simplex::top_dimension_cofaces(*m_cut_mesh, v_on_cut_mesh).simplex_vector();
        for (const auto& s : triangle_simplex_to_check) {
            triangle_tuples_to_check.emplace_back(s.tuple());
        }
    }

    /////////////////////////////////////////////////////////////////////////////////
    // get all the rotation matrix
    // and the index of the vertex copy in the vs_on_cut_mesh that the edge is rotated to
    /////////////////////////////////////////////////////////////////////////////////
    auto find_next_bd_edge = [this](const Tuple input_edge_tuple) -> Tuple {
        Tuple cur_edge = input_edge_tuple;
        cur_edge = this->m_cut_mesh->switch_edge(cur_edge);

        while (!this->m_cut_mesh->is_boundary(Simplex(PrimitiveType::Edge, cur_edge))) {
            cur_edge = this->m_cut_mesh->switch_face(cur_edge);
            cur_edge = this->m_cut_mesh->switch_edge(cur_edge);
        }
        return cur_edge;
    };
    std::vector<Eigen::Matrix<double, 2, 2>> rotation_matrix;
    std::vector<int> rotate_to;
    Tuple cur_edge_tuple = vs_on_cut_mesh.front().tuple();
    cur_edge_tuple = find_next_bd_edge(cur_edge_tuple);
    do {
        Simplex cur_edge = Simplex(PrimitiveType::Edge, cur_edge_tuple);
        Simplex pair_edge = wmtk::utils::get_pair_edge(mesh(), *m_cut_mesh, cur_edge);
        Eigen::Matrix<double, 2, 2> rotation_matrix_i =
            wmtk::utils::get_rotation_matrix(*m_cut_mesh, m_uv_handle, cur_edge, pair_edge);
        rotation_matrix.emplace_back(rotation_matrix_i);
        Tuple pair_edge_tuple = pair_edge.tuple();
        for (int i = 0; i < vs_on_cut_mesh.size(); ++i) {
            if (wmtk::simplex::utils::SimplexComparisons::equal(
                    *m_cut_mesh,
                    Simplex(PrimitiveType::Vertex, pair_edge_tuple),
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
            return false;
        } else {
            // wmtk::logger().info("singular vertex not detected");
        }
    }
    /////////////////////////////////////////////////////////////////////////////////
    // get search direction for each vertex copy
    /////////////////////////////////////////////////////////////////////////////////
    // get the gradient descent direction for each vertex copy
    std::vector<Eigen::VectorXd> gradient_dir;
    for (const auto& v_on_cut_mesh : vs_on_cut_mesh) {
        gradient_dir.emplace_back(-m_settings.energy->get_gradient(v_on_cut_mesh).head<2>());
    }

    // rotate the gradient descent direction and sum up to get the search direction for the first
    // copy
    std::vector<Eigen::VectorXd> search_dir = gradient_dir;
    Eigen::Matrix<double, 2, 2> current_rotation_matrix;
    current_rotation_matrix.setIdentity();
    for (int i = 0; i < rotation_matrix.size() - 1; ++i) {
        current_rotation_matrix = current_rotation_matrix * rotation_matrix[i].transpose();

        search_dir[0] += current_rotation_matrix * gradient_dir[rotate_to[i]];
    }
    // rotate the search direction for the first copy to get the search direction for the rest
    for (int i = 0; i < rotation_matrix.size() - 1; ++i) {
        if (i == 0) {
            search_dir[rotate_to[i]] = rotation_matrix[i] * search_dir[0];
        } else {
            search_dir[rotate_to[i]] = rotation_matrix[i] * search_dir[rotate_to[i - 1]];
        }
    }
    // TODO: won't need it if the attribute is a 2d vector
    for (int i = 0; i < vs_on_cut_mesh.size(); ++i) {
        search_dir[i].conservativeResize(3);
        search_dir[i](2) = 0;
    }


    /////////////////////////////////////////////////////////////////////////////////
    // Do line search here!!!
    /////////////////////////////////////////////////////////////////////////////////
    auto get_energy_value = [&]() -> double {
        double energy = 0;
        for (const auto& v_on_cut_mesh : vs_on_cut_mesh) {
            energy += m_settings.energy->get_value(v_on_cut_mesh);
        }
        return energy;
    };

    double step_size = 1.0; // TODO: set as a parameter
    int max_steps = 50; //  TODO: set as a parameter
    int steps = 0;
    double energy_before = get_energy_value();
    double current_energy = energy_before;
    std::vector<Eigen::Vector3d> current_uvs;
    for (const auto& v_on_cut_mesh : vs_on_cut_mesh) {
        current_uvs.emplace_back(uv_accessor.vector_attribute(v_on_cut_mesh.tuple()));
    }

    do {
        for (int i = 0; i < vs_on_cut_mesh.size(); ++i) {
            uv_accessor.vector_attribute(vs_on_cut_mesh[i].tuple()) =
                current_uvs[i] + step_size * search_dir[i];
        }
        current_energy = get_energy_value();

        // wmtk::logger().info("energy before: {}", energy_before);
        // wmtk::logger().info("step_size: {}", step_size);
        // wmtk::logger().info("energy after: {}", current_energy);

        if (!std::isnan(current_energy) && !std::isinf(current_energy) &&
            current_energy < energy_before) {
            if (cut_mesh_inversion_check.after(PrimitiveType::Face, triangle_tuples_to_check)) {
                // wmtk::logger().info("opt success on boundary vertex");
                return true;
            } else {
                // wmtk::logger().info("inversion detected");
            }
        }

        step_size /= 2;
    } while (steps++ < max_steps);

    return false;
}


bool SeamlessGradientDescent::execute()
{
    // 1. map v_on_seamed_mesh to vs_on_cut_mesh
    std::vector<Simplex> vs_on_cut_mesh =
        mesh().map_to_child(*m_cut_mesh, Simplex(PrimitiveType::Vertex, input_tuple()));

    if (m_cut_mesh->is_boundary(vs_on_cut_mesh.front())) {
        if (vs_on_cut_mesh.size() == 1) {
            // Can't smooth leave nodes
            return false;
        }
        if (!execute_on_boundary(vs_on_cut_mesh)) {
            return false;
        }
    } else {
        assert(vs_on_cut_mesh.size() == 1);
        if (!execute_on_interior(vs_on_cut_mesh.front())) {
            return false;
        }
    }

    m_output_tuple = input_tuple();

    return true;
}

// TODO: maybe need to rewrite this
// std::vector<double> SeamlessGradientDescent::priority() const
// {
//     // double gradnorm = m_settings.energy->get_gradient(Simplex::vertex(input_tuple())).norm();
//     // std::vector<double> r;
//     // r.emplace_back(-gradnorm);
//     // return r;
// }
} // namespace wmtk::operations::tri_mesh::internal
