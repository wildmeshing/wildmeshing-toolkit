#include "SeamlessGradientDescent.hpp"
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>
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
    Simplex v_on_seamed_mesh = Simplex(PrimitiveType::Vertex, input_tuple());

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

    // TODO: implement this
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

        // wmtk::logger().info("energy before: {}", energy_before);
        // wmtk::logger().info("step_size: {}", step_size);
        // wmtk::logger().info("energy after: {}", current_energy);

        current_energy = m_settings.energy->get_value(v_on_cut_mesh);
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
    // Simplex v_on_seamed_mesh = Simplex(PrimitiveType::Vertex, input_tuple());

    // auto pos_accessor = coordinate_accessor();
    // auto uv_accessor = m_cut_mesh->create_accessor(m_uv_handle);
    // TODO: implement this
    return true;
}


bool SeamlessGradientDescent::execute()
{
    // 1. map v_on_seamed_mesh to vs_on_cut_mesh
    std::vector<Simplex> vs_on_cut_mesh =
        mesh().map_to_child(*m_cut_mesh, Simplex(PrimitiveType::Vertex, input_tuple()));

    if (vs_on_cut_mesh.size() == 1) {
        // wmtk::logger().info("vs_on_cut_mesh.size() == 1");
        // it means that the vertex is in the interior of the cut mesh
        if (!execute_on_interior(vs_on_cut_mesh.front())) {
            return false;
        }
    } else {
        // it means that the vertex is on the boundary of the cut mesh
        if (!execute_on_boundary(vs_on_cut_mesh)) {
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
