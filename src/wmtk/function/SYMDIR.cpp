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


DScalar::Scalar SYMDIR::get_energy_avg() const
{
    const auto all_face_tuples = mesh().get_all(PrimitiveType::Face);
    std::vector<Simplex> all_faces;
    for (const auto& face_tuple : all_face_tuples) {
        all_faces.push_back(Simplex::face(face_tuple));
    }
    const auto energy_sum = get_value_sum(all_faces);

    if (m_do_integral) {
        DScalar::Scalar area_sum = 0;
        for (const auto& face_simplex : all_faces) {
            std::vector<Eigen::Vector3<DScalar::Scalar>> ref_coordinates(3);
            {
                auto m_vertex_attribute_handle = m_vertex_attribute_handle_opt.value();
                ConstAccessor<double> ref_accessor =
                    m_ref_mesh.create_const_accessor(m_vertex_attribute_handle);
                const simplex::Simplex ref_domain_simplex = mesh().map_to_parent(face_simplex);

                const std::vector<Tuple> faces = wmtk::simplex::faces_single_dimension_tuples(
                    m_ref_mesh,
                    ref_domain_simplex,
                    PrimitiveType::Vertex);


                assert(faces.size() == 3);
                for (long i = 0; i < 3; ++i) {
                    auto value = ref_accessor.const_vector_attribute(faces[i]).eval();
                    ref_coordinates[i] = value.cast<DScalar::Scalar>();
                }
            }

            auto area = (wmtk::utils::triangle_3d_area(
                             ref_coordinates[0],
                             ref_coordinates[1],
                             ref_coordinates[2]) +
                         wmtk::utils::triangle_3d_area(
                             ref_coordinates[1],
                             ref_coordinates[2],
                             ref_coordinates[0]) +
                         wmtk::utils::triangle_3d_area(
                             ref_coordinates[2],
                             ref_coordinates[0],
                             ref_coordinates[1])) /
                        3;

            area_sum += area;
        }

        return energy_sum / area_sum;
    } else {
        return get_value_sum(all_faces) / all_faces.size();
    }
}

DScalar::Scalar SYMDIR::get_energy_max() const
{
    const auto all_face_tuples = mesh().get_all(PrimitiveType::Face);
    std::vector<Simplex> all_faces;
    for (const auto& face_tuple : all_face_tuples) {
        all_faces.push_back(Simplex::face(face_tuple));
    }
    return get_value_max(all_faces);
}

DScalar SYMDIR::eval(const simplex::Simplex& domain_simplex, const std::array<DSVec, 3>& coords)
    const
{
    std::vector<Eigen::Vector3<DScalar>> ref_coordinates(3); // reference triangle coordinates

    if (m_uniform_reference) {
        // std::cout << "uniform reference" << std::endl;
        ref_coordinates[0] << DScalar(0.), DScalar(0.), DScalar(0.);
        ref_coordinates[1] << DScalar(1.), DScalar(0.), DScalar(0.);
        ref_coordinates[2] << DScalar(0.5), DScalar(sqrt(3) / 2.), DScalar(0.);
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
            ref_coordinates[i] = value.cast<DScalar>();
        }
    }


    // TODO: After fixing readmsh check dimension
    // switch (embedded_dimension()) {
    // case 2: {

    DSVec2 a = coords[0].head<2>(), b = coords[1].head<2>(), c = coords[2].head<2>();

    // auto energy_density =
    //     (utils::symdir(ref_coordinates[0], ref_coordinates[1], ref_coordinates[2], a, b, c) +
    //      utils::symdir(ref_coordinates[1], ref_coordinates[2], ref_coordinates[0], b, c, a) +
    //      utils::symdir(ref_coordinates[2], ref_coordinates[0], ref_coordinates[1], c, a, b)) /
    //     3;
    auto energy_density =
        (utils::symdir(ref_coordinates[0], ref_coordinates[1], ref_coordinates[2], a, b, c) +
         utils::symdir(ref_coordinates[1], ref_coordinates[2], ref_coordinates[0], b, c, a) +
         utils::symdir(ref_coordinates[2], ref_coordinates[0], ref_coordinates[1], c, a, b) +
         utils::symdir(ref_coordinates[2], ref_coordinates[1], ref_coordinates[0], c, b, a) +
         utils::symdir(ref_coordinates[1], ref_coordinates[0], ref_coordinates[2], b, a, c) +
         utils::symdir(ref_coordinates[0], ref_coordinates[2], ref_coordinates[1], a, c, b)) /
        6;


    if (m_do_integral) {
        auto area = (wmtk::utils::triangle_3d_area(
                         ref_coordinates[0],
                         ref_coordinates[1],
                         ref_coordinates[2]) +
                     wmtk::utils::triangle_3d_area(
                         ref_coordinates[1],
                         ref_coordinates[2],
                         ref_coordinates[0]) +
                     wmtk::utils::triangle_3d_area(
                         ref_coordinates[2],
                         ref_coordinates[0],
                         ref_coordinates[1])) /
                    3;
        return area * energy_density;
    } else {
        return energy_density;
    }
    // }
    // default: throw std::runtime_error("Symmetric Dirichlet energy is only defined in 2d");
    // }
}

} // namespace wmtk::function
