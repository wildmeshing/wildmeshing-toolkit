#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <tools/DEBUG_TetMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/TetMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/mesh_decimation/internal/MeshDecimation.hpp>
#include <wmtk/components/mesh_decimation/mesh_decimation.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/utils/mesh_utils.hpp>


#include "wmtk/Scheduler.hpp"
#include "wmtk/function/simplex/AMIPS.hpp"
#include "wmtk/invariants/SimplexInversionInvariant.hpp"
#include "wmtk/invariants/SmallerFunctionInvariant.hpp"
#include "wmtk/invariants/TodoInvariant.hpp"
#include "wmtk/operations/EdgeCollapse.hpp"
#include "wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp"
#include "wmtk/operations/attribute_update/AttributeTransferStrategy.hpp"

using json = nlohmann::json;
using namespace wmtk;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("preprocess", "[.]")
{
    using namespace wmtk::components;

    auto mesh_in = wmtk::read_mesh(data_dir / "3d_images/sphere_regularized.hdf5");
    Mesh& mesh = static_cast<Mesh&>(*mesh_in);

    std::vector<wmtk::attribute::MeshAttributeHandle> pass_though;
    wmtk::attribute::MeshAttributeHandle vertex =
        mesh.get_attribute_handle<int64_t>("vertex_tag", PrimitiveType::Vertex);
    wmtk::attribute::MeshAttributeHandle edge =
        mesh.get_attribute_handle<int64_t>("edge_tag", PrimitiveType::Edge);
    wmtk::attribute::MeshAttributeHandle face =
        mesh.get_attribute_handle<int64_t>("face_tag", PrimitiveType::Triangle);
    pass_though.push_back(vertex);
    pass_though.push_back(edge);
    pass_though.push_back(face);
    internal::MeshDecimation MD(mesh, "tag", 1, 5, pass_though);
    MD.process();

    wmtk::io::ParaviewWriter
        writer(data_dir / "3d_images/out.hdf", "vertices", mesh, false, true, true, false);
    mesh.serialize(writer);

    // wmtk::io::Cache cache("wmtk_cache", ".");
    // cache.write_mesh(mesh, data_dir / "3d_images/kinderout");
}

TEST_CASE("preprocessss", "[.]")
{
    using namespace wmtk::attribute;
    using namespace wmtk::invariants;
    double target_len = 0.00026;
    auto mesh_in = wmtk::read_mesh(data_dir / "3d_images/sphere_regularized.hdf5");
    TetMesh& mesh = static_cast<TetMesh&>(*mesh_in);

    wmtk::attribute::MeshAttributeHandle tet_handle =
        mesh.get_attribute_handle<int64_t>("tag", PrimitiveType::Tetrahedron);

    wmtk::attribute::MeshAttributeHandle position =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    wmtk::attribute::MeshAttributeHandle edge_handle =
        mesh.register_attribute<int64_t>("todo_edge_", PrimitiveType::Edge, 1);

    wmtk::attribute::MeshAttributeHandle vertex_handle =
        mesh.register_attribute<int64_t>("todo_vertex_", PrimitiveType::Vertex, 1);

    wmtk::attribute::MeshAttributeHandle edge_len_handle =
        mesh.register_attribute<double>("len_edge_", PrimitiveType::Edge, 1);

    Accessor<int64_t> acc_tet = mesh.create_accessor<int64_t>(tet_handle);
    Accessor<int64_t> acc_edge = mesh.create_accessor<int64_t>(edge_handle);
    Accessor<int64_t> acc_vertex = mesh.create_accessor<int64_t>(vertex_handle);
    Accessor<double> acc_pos = mesh.create_accessor<double>(position);
    Accessor<double> acc_len = mesh.create_accessor<double>(edge_len_handle);

    for (const Tuple& face : mesh.get_all(PrimitiveType::Triangle)) {
        if (mesh.is_boundary_face(face)) {
            acc_vertex.scalar_attribute(face) = 1;
            acc_vertex.scalar_attribute(mesh.switch_vertex(face)) = 1;
            acc_vertex.scalar_attribute(mesh.switch_vertex(mesh.switch_edge(face))) = 1;

            acc_edge.scalar_attribute(face) = 1;
            acc_edge.scalar_attribute(mesh.switch_edge(face)) = 1;
            acc_edge.scalar_attribute(mesh.switch_edge(mesh.switch_vertex(face))) = 1;
        } else if (
            acc_tet.scalar_attribute(face) !=
            acc_tet.scalar_attribute(mesh.switch_tetrahedron(face))) {
            acc_vertex.scalar_attribute(face) = 1;
            acc_vertex.scalar_attribute(mesh.switch_vertex(face)) = 1;
            acc_vertex.scalar_attribute(mesh.switch_vertex(mesh.switch_edge(face))) = 1;

            acc_edge.scalar_attribute(face) = 1;
            acc_edge.scalar_attribute(mesh.switch_edge(face)) = 1;
            acc_edge.scalar_attribute(mesh.switch_edge(mesh.switch_vertex(face))) = 1;
        }
    }

    for (const Tuple& edge : mesh.get_all(PrimitiveType::Edge)) {
        if (acc_vertex.scalar_attribute(edge) == 1 ||
            acc_vertex.scalar_attribute(mesh.switch_vertex(edge)) == 1) {
            acc_edge.scalar_attribute(edge) = 1;
        }
        acc_len.scalar_attribute(edge) =
            (acc_pos.vector_attribute(edge) - acc_pos.vector_attribute(mesh.switch_vertex(edge)))
                .norm();
    }

    auto op_scaffold = std::make_shared<operations::EdgeCollapse>(mesh);

    op_scaffold->add_invariant(std::make_shared<TodoInvariant>(mesh, edge_handle.as<int64_t>(), 0));
    op_scaffold->add_invariant(std::make_shared<TodoSmallerInvariant>(
        mesh,
        edge_len_handle.as<double>(),
        4.0 / 5.0 * target_len));

    auto m_amips = std::make_shared<function::AMIPS>(mesh, position);
    auto m_link_conditions = std::make_shared<invariants::InvariantCollection>(mesh);
    auto m_function_invariant = std::make_shared<invariants::SmallerFunctionInvariant>(
        mesh.top_simplex_type(),
        m_amips,
        30);
    auto m_inversion_invariant =
        std::make_shared<SimplexInversionInvariant>(mesh, position.as<double>());

    // Edge length update
    auto compute_edge_length = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorXd::Constant(1, (P.col(0) - P.col(1)).norm());
    };
    auto m_edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            edge_len_handle,
            position,
            compute_edge_length);
    auto m_prio_short_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        auto acc = mesh.create_accessor<double>(edge_len_handle);
        return std::vector<double>({acc.scalar_attribute(s.tuple())});
    };


    op_scaffold->add_invariant(m_link_conditions);
    op_scaffold->add_invariant(m_function_invariant);
    op_scaffold->add_invariant(m_inversion_invariant);

    op_scaffold->add_transfer_strategy(m_edge_length_update);

    op_scaffold->set_priority(m_prio_short_edges_first);

    op_scaffold->set_new_attribute_strategy(
        position,
        wmtk::operations::CollapseBasicStrategy::Mean);
    op_scaffold->set_new_attribute_strategy(vertex_handle);

    op_scaffold->set_new_attribute_strategy(edge_handle);
    op_scaffold->set_new_attribute_strategy(edge_len_handle);
    op_scaffold->set_new_attribute_strategy(tet_handle);

    while (true) {
        Scheduler scheduler;
        SchedulerStats pass_stats = scheduler.run_operation_on_all(*op_scaffold);
        if (pass_stats.number_of_successful_operations() == 0) break;
    }

    wmtk::io::ParaviewWriter
        writer(data_dir / "3d_images/kinderout.hdf", "vertices", mesh, false, true, true, false);
    mesh.serialize(writer);

    wmtk::io::Cache cache("wmtk_cache", ".");
    cache.write_mesh(mesh, data_dir / "3d_images/kinderout");
}