#include "ExtremeOptCollapse.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/invariants/find_invariant_in_collection_by_type.hpp>

#include <cmath>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/MaxEdgeLengthInvariant.hpp>
#include <wmtk/invariants/TriangleDegenerateInvariant.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>
namespace wmtk::operations {

void OperationSettings<tri_mesh::ExtremeOptCollapse>::create_invariants()
{
    OperationSettings<tri_mesh::EdgeCollapse>::create_invariants();

    // TODO: need to discuss if we still want this invariant
    invariants->add(
        std::make_shared<MaxEdgeLengthInvariant>(*uv_mesh_ptr, uv_handle, max_squared_length));
    invariants->add(std::make_shared<TriangleInversionInvariant>(*uv_mesh_ptr, uv_handle));
    invariants->add(std::make_shared<TriangleDegenerateInvariant>(m_mesh, position));
}


namespace tri_mesh {
ExtremeOptCollapse::ExtremeOptCollapse(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<ExtremeOptCollapse>& settings)
    : EdgeCollapse(m, t, settings)
    , m_pos_accessor{m.create_accessor(settings.position)}
    , m_uv_accessor{settings.uv_mesh_ptr->create_accessor(settings.uv_handle)}
    , m_settings{settings}
{
    if (m_settings.optimize_E_max) {
        symdir_ptr = std::make_shared<wmtk::function::SYMDIR>(
            mesh(),
            *m_settings.uv_mesh_ptr,
            m_settings.position,
            m_settings.uv_handle,
            false);
    } else {
        // we optimize E_sum, need to do integral here
        symdir_ptr = std::make_shared<wmtk::function::SYMDIR>(
            mesh(),
            *m_settings.uv_mesh_ptr,
            m_settings.position,
            m_settings.uv_handle,
            true);
    }
}

std::vector<double> ExtremeOptCollapse::priority() const
{
    const auto input_tuple_uv =
        mesh().map_to_child_tuples(*m_settings.uv_mesh_ptr, Simplex::edge(input_tuple())).front();
    auto uv0 = m_uv_accessor.vector_attribute(input_tuple_uv);
    auto uv1 =
        m_uv_accessor.vector_attribute(m_settings.uv_mesh_ptr->switch_vertex(input_tuple_uv));
    return {(uv0 - uv1).norm()};
}

std::string ExtremeOptCollapse::name() const
{
    return "tri_mesh_collapse_edge_to_mid_extreme_opt";
}


bool ExtremeOptCollapse::before() const
{
    return TupleOperation::before();
}

std::tuple<
    std::vector<Tuple>,
    std::vector<Tuple>,
    std::vector<Eigen::VectorXd>,
    std::vector<Eigen::VectorXd>>
ExtremeOptCollapse::cache_data_before_execute() const
{
    // cache the uv coordinates of the endpoints
    const auto input_tuples_uv =
        mesh().map_to_child_tuples(*m_settings.uv_mesh_ptr, Simplex::edge(input_tuple()));
    std::vector<Tuple> output_tuples_uv; // store one of the ear edge on the neighbor triangle, need
                                         // to ressurect it later
    std::vector<Eigen::VectorXd> coord0s_uv;
    std::vector<Eigen::VectorXd> coord1s_uv;
    for (const auto& input_tuple_uv : input_tuples_uv) {
        // two candidate ear edges
        const Tuple ear_0 = m_settings.uv_mesh_ptr->switch_edge(input_tuple_uv);
        const Tuple ear_1 = m_settings.uv_mesh_ptr->switch_tuples(
            input_tuple_uv,
            {PrimitiveType::Vertex, PrimitiveType::Edge});
        // choose the one that is not a boundary edge then switch_face to the neighbor triangle
        if (m_settings.uv_mesh_ptr->is_boundary_edge(ear_0)) {
            assert(!m_settings.uv_mesh_ptr->is_boundary_edge(ear_1));
            output_tuples_uv.push_back(m_settings.uv_mesh_ptr->switch_face(ear_1));
        } else {
            output_tuples_uv.push_back(m_settings.uv_mesh_ptr->switch_face(ear_0));
        }
        coord0s_uv.push_back(m_uv_accessor.vector_attribute(input_tuple_uv));
        coord1s_uv.push_back(
            m_uv_accessor.vector_attribute(m_settings.uv_mesh_ptr->switch_vertex(input_tuple_uv)));
    }
    return std::make_tuple(input_tuples_uv, output_tuples_uv, coord0s_uv, coord1s_uv);
}

bool ExtremeOptCollapse::check_branch_vertex_invartiant(const Tuple& input_tuple_uv, bool& keep_v0)
    const
{
    bool v0_is_boundary = false;
    bool v1_is_boundary = false;

    // check if the endpoints are boundary vertices
    // we always collapse towards the boundary if possbile
    v0_is_boundary = m_settings.uv_mesh_ptr->is_boundary_vertex(input_tuple_uv);
    v1_is_boundary = m_settings.uv_mesh_ptr->is_boundary_vertex(
        m_settings.uv_mesh_ptr->switch_vertex(input_tuple_uv));

    if (v1_is_boundary && !v0_is_boundary) {
        keep_v0 = false;
    } else if (v0_is_boundary && v1_is_boundary) {
        // special invariant for ExtremeOptCollapse: keep the branch vertices
        const auto input_tuples_uv_v0 =
            mesh().map_to_child_tuples(*m_settings.uv_mesh_ptr, Simplex::vertex(input_tuple()));
        const auto input_tuples_uv_v1 = mesh().map_to_child_tuples(
            *m_settings.uv_mesh_ptr,
            Simplex::vertex(mesh().switch_vertex(input_tuple())));
        if (input_tuples_uv_v0.size() != 2 && input_tuples_uv_v1.size() != 2) {
            return false; // both are branch vertices, do not collapse
        } else if (input_tuples_uv_v1.size() != 2) {
            keep_v0 = false; // v1 is branch vertex, keep v0
        }
    }

    return true;
}

double ExtremeOptCollapse::get_energy_before() const
{
    double energy_before;

    const auto v0_neighbor_faces =
        wmtk::simplex::top_dimension_cofaces(mesh(), Simplex(PrimitiveType::Vertex, input_tuple()));

    const auto v1_neighbor_faces = wmtk::simplex::top_dimension_cofaces(
        mesh(),
        Simplex(PrimitiveType::Vertex, mesh().switch_vertex(input_tuple())));

    std::vector<Simplex> eval_domain_faces =
        wmtk::simplex::SimplexCollection::get_union(v0_neighbor_faces, v1_neighbor_faces)
            .simplex_vector();

    // map to uv_mesh
    std::vector<Simplex> eval_domain_faces_uv;
    for (long i = 0; i < eval_domain_faces.size(); ++i) {
        eval_domain_faces_uv.push_back(
            mesh().map_to_child(*m_settings.uv_mesh_ptr, eval_domain_faces[i]).front());
    }
    if (m_settings.optimize_E_max) {
        energy_before = symdir_ptr->get_value_max(eval_domain_faces_uv);
    } else {
        energy_before = symdir_ptr->get_value_sum(eval_domain_faces_uv);
    }
    return energy_before;
}

double ExtremeOptCollapse::get_energy_after() const
{
    double energy_after;

    // const std::vector<Tuple> eval_domain_face_tuples = EdgeCollapse::modified_triangles();
    const std::vector<Simplex> eval_domain_face_simplex =
        wmtk::simplex::top_dimension_cofaces(
            mesh(),
            Simplex(PrimitiveType::Vertex, EdgeCollapse::return_tuple()))
            .simplex_vector();

    // map to uv_mesh
    std::vector<Simplex> eval_domain_faces_uv;
    for (long i = 0; i < eval_domain_face_simplex.size(); ++i) {
        eval_domain_faces_uv.push_back(
            mesh().map_to_child(*m_settings.uv_mesh_ptr, eval_domain_face_simplex[i]).front());
    }
    if (m_settings.optimize_E_max) {
        energy_after = symdir_ptr->get_value_max(eval_domain_faces_uv);
    } else {
        energy_after = symdir_ptr->get_value_sum(eval_domain_faces_uv);
    }

    return energy_after;
}

bool ExtremeOptCollapse::execute()
{
    // cache endpoint data on 3d mesh
    auto p0 = m_pos_accessor.vector_attribute(input_tuple()).eval();
    auto p1 = m_pos_accessor.vector_attribute(mesh().switch_vertex(input_tuple())).eval();

    // cache endpoint data on uv mesh
    auto [input_tuples_uv, output_tuples_uv, coord0s_uv, coord1s_uv] = cache_data_before_execute();
    assert(input_tuples_uv.size() > 0);
    assert(input_tuples_uv.size() < 3);

    // decide which endpoint to keep, and check branch vertex invariant
    bool keep_v0 = true;
    if (!check_branch_vertex_invartiant(input_tuples_uv.front(), keep_v0)) {
        return false;
    }

    double energy_before_collapse = get_energy_before();

    if (std::isinf(energy_before_collapse) || std::isnan(energy_before_collapse)) {
        std::cout << "what the fuck?" << std::endl;
    }

    // execute collapse
    {
        if (!EdgeCollapse::execute()) {
            return false;
        }
    }


    // update the position on 3d mesh
    const Tuple collapse_output_tuple = EdgeCollapse::return_tuple();
    if (keep_v0) {
        m_pos_accessor.vector_attribute(collapse_output_tuple) = p0;
    } else {
        m_pos_accessor.vector_attribute(collapse_output_tuple) = p1;
    }

    // resurrect the output_tuples_uv on uv_mesh
    for (Tuple& output_tuple_uv : output_tuples_uv) {
        output_tuple_uv = resurrect_tuple(
            *m_settings.uv_mesh_ptr,
            output_tuple_uv); // new added helper function in Operation.hpp
    }

    // update the uv coordinates on uv mesh
    for (size_t i = 0; i < output_tuples_uv.size(); ++i) {
        if (keep_v0) {
            m_uv_accessor.vector_attribute(output_tuples_uv[i]) = coord0s_uv[i];
        } else {
            m_uv_accessor.vector_attribute(output_tuples_uv[i]) = coord1s_uv[i];
        }
    }

    double energy_after_collapse = get_energy_after();

    if (std::isinf(energy_after_collapse) || std::isnan(energy_after_collapse)) {
        return false;
    }
    if (energy_after_collapse >= energy_before_collapse) {
        return false;
    }

    return true;
}

} // namespace tri_mesh
} // namespace wmtk::operations
