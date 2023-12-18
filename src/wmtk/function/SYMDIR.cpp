#include "SYMDIR.hpp"
#include <wmtk/TriMesh.hpp>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/symdir.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/triangle_areas.hpp>

namespace wmtk::function {
SYMDIR::SYMDIR(const TriMesh& uv_mesh, const MeshAttributeHandle<double>& uv_attribute_handle)
    : TriangleAutodiffFunction(uv_mesh, uv_attribute_handle)
    , m_ref_mesh(uv_mesh)
{
    m_uniform_reference = true;
    m_do_integral = false;
}

SYMDIR::SYMDIR(
    const TriMesh& ref_mesh,
    const TriMesh& uv_mesh,
    const MeshAttributeHandle<double>& vertex_attribute_handle,
    const MeshAttributeHandle<double>& uv_attribute_handle,
    bool do_integral)
    : TriangleAutodiffFunction(uv_mesh, uv_attribute_handle)
    , m_ref_mesh(ref_mesh)
    , m_vertex_attribute_handle_opt(vertex_attribute_handle)
    , m_do_integral(do_integral)
{
    m_uniform_reference = false;
}

SYMDIR::~SYMDIR() = default;

using DScalar = typename AutodiffFunction::DScalar;
using DSVec2 = Eigen::Vector2<DScalar>;
DScalar SYMDIR::eval(const simplex::Simplex& domain_simplex, const std::array<DSVec, 3>& coords)
    const
{
    std::vector<Eigen::Vector3<DScalar>> ref_coordinaites(3); // reference triangle coordinates

    if (m_uniform_reference) {
        // std::cout << "uniform reference" << std::endl;
        ref_coordinaites[0] << DScalar(0.), DScalar(0.), DScalar(0.);
        ref_coordinaites[1] << DScalar(1.), DScalar(0.), DScalar(0.);
        ref_coordinaites[2] << DScalar(0.5), DScalar(sqrt(3) / 2.), DScalar(0.);
    } else {
        // std::cout << "non-uniform reference" << std::endl;
        assert(m_vertex_attribute_handle_opt.has_value());
        auto m_vertex_attribute_handle = m_vertex_attribute_handle_opt.value();
        ConstAccessor<double> ref_accessor =
            m_ref_mesh.create_const_accessor(m_vertex_attribute_handle);
        const simplex::Simplex ref_domain_simplex = mesh().map_to_parent(domain_simplex);

        const std::vector<Tuple> faces = wmtk::simplex::faces_single_dimension_tuples(
            m_ref_mesh,
            ref_domain_simplex,
            PrimitiveType::Vertex);

        assert(faces.size() == 3);
        for (long i = 0; i < 3; ++i) {
            auto value = ref_accessor.const_vector_attribute(faces[i]).eval();
            ref_coordinaites[i] = value.cast<DScalar>();
        }
    }

    switch (embedded_dimension()) {
    case 2: {
        DSVec2 a = coords[0], b = coords[1], c = coords[2];
        if (m_do_integral) {
            // if do integral, multiplied by the area of the triangle
            return wmtk::utils::triangle_3d_area(
                       ref_coordinaites[0],
                       ref_coordinaites[1],
                       ref_coordinaites[2]) *
                   utils::symdir(
                       ref_coordinaites[0],
                       ref_coordinaites[1],
                       ref_coordinaites[2],
                       a,
                       b,
                       c);
        }
        return utils::symdir(
            ref_coordinaites[0],
            ref_coordinaites[1],
            ref_coordinaites[2],
            a,
            b,
            c);
    }
    default: throw std::runtime_error("Symmetric Dirichlet energy is only defined in 2d");
    }
}

} // namespace wmtk::function
