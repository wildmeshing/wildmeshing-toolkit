#include "VertexSmoothUsingDifferentiableEnergy.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk/utils/triangle_helper_functions.hpp>
#include "VertexSmooth.hpp"

namespace wmtk::operations {
void OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy>::initialize_invariants(
    const TriMesh& m)
{
    smooth_settings.initialize_invariants(m);
    smooth_settings.invariants.add(
        std::make_shared<TriangleInversionInvariant>(m, smooth_settings.position));
}
} // namespace wmtk::operations

namespace wmtk::operations::tri_mesh {
VertexSmoothUsingDifferentiableEnergy::VertexSmoothUsingDifferentiableEnergy(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings)
    : TriMeshOperation(m)
    , TupleOperation(settings.smooth_settings.invariants, t)
    , m_uv_pos_accessor{m.create_accessor<double>(settings.smooth_settings.position)}
    , m_settings{settings}
{}

std::string VertexSmoothUsingDifferentiableEnergy::name() const
{
    return "tri_mesh_vertex_smooth_using_differentiable_energy";
}

bool VertexSmoothUsingDifferentiableEnergy::before() const
{
    if (!mesh().is_valid_slow(input_tuple())) {
        return false;
    }
    return true;
}

bool VertexSmoothUsingDifferentiableEnergy::execute()
{
    const Eigen::Vector2d p = m_uv_pos_accessor.vector_attribute(input_tuple());
    OperationSettings<tri_mesh::VertexSmooth> op_settings;
    tri_mesh::VertexSmooth smooth_op(mesh(), input_tuple(), m_settings.smooth_settings);
    if (!smooth_op()) {
        return false;
    }

    const Tuple tup = smooth_op.return_tuple();
    Eigen::Vector2d new_pos = p;

    assert(mesh().is_valid_slow(tup));
    // start scope
    // auto scope = m_mesh.create_scope();

    if (m_settings.smooth_settings.smooth_boundary && mesh().is_boundary_vertex(tup)) {
        // do curve mesh smoothing

    } else {
        // get one ring energy wrt to the vertex
        const std::vector<Simplex> one_ring =
            SimplicialComplex::vertex_one_ring(mesh(), input_tuple());
        double total_energy = 0;
        Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
        Eigen::Matrix2d hessian = Eigen::Matrix2d::Zero();
        for (const Simplex& s : one_ring) {
            total_energy += m_settings.energy->get_value(s.tuple());
            gradient += m_settings.energy->get_gradient(s.tuple());
            if (m_settings.second_order) hessian += m_settings.energy->get_hessian(s.tuple());
        }
        // get descent direction
        Eigen::Vector2d descent_dir = Eigen::Vector2d::Zero();
        if (m_settings.second_order) {
            // newton's method
            descent_dir = -hessian.ldlt().solve(gradient);
        } else {
            // gradient descent
            descent_dir = -gradient;
        }
        new_pos = p + descent_dir;
        // set new position
        m_uv_pos_accessor.vector_attribute(tup) = new_pos;
        // check if the new position is valid
        // for (const Simplex& s : one_ring) {
        //     if (triangle_2d_area(s.tuple()) < 0) {
        //         scope.mark_failed();
        //         break;
        //     }
        // }
    }

    return true;
}


} // namespace wmtk::operations::tri_mesh
