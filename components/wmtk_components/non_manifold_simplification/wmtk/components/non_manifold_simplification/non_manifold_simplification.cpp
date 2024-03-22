#include "non_manifold_simplification.hpp"

#include <wmtk/components/base/get_attributes.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/Scheduler.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

#include <wmtk/invariants/EnvelopeInvariant.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MaxEdgeLengthInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>

#include <wmtk/operations/AttributesUpdate.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/operations/composite/ProjectOperation.hpp>
#include <wmtk/operations/utils/VertexLaplacianSmooth.hpp>

#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/link_single_dimension.hpp>

#include "NonManifoldSimplificationOptions.hpp"

namespace {
double relative_to_absolute_length(
    const wmtk::attribute::MeshAttributeHandle& pos_handle,
    const double length_rel)
{
    auto pos = pos_handle.mesh().create_const_accessor<double>(pos_handle);
    const auto vertices = pos_handle.mesh().get_all(wmtk::PrimitiveType::Vertex);
    Eigen::AlignedBox<double, Eigen::Dynamic> bbox(pos.dimension());


    for (const auto& v : vertices) {
        bbox.extend(pos.const_vector_attribute(v));
    }

    const double diag_length = bbox.sizes().norm();

    return length_rel * diag_length;
}
} // namespace

namespace wmtk::components {

void write(
    const Mesh& mesh,
    const std::string& name,
    const int64_t index,
    const bool intermediate_output)
{
    if (intermediate_output) {
        // write trimesh
        const std::filesystem::path data_dir = "";
        wmtk::io::ParaviewWriter writer(
            data_dir / (name + "_" + std::to_string(index)),
            "vertices",
            mesh,
            false,
            mesh.top_simplex_type() == PrimitiveType::Edge,
            mesh.top_simplex_type() == PrimitiveType::Triangle,
            mesh.top_simplex_type() == PrimitiveType::Tetrahedron);
        mesh.serialize(writer);
    }
}

void non_manifold_simplification(
    const base::Paths& paths,
    const nlohmann::json& j,
    io::Cache& cache)
{
    NonManifoldSimplificationOptions options = j.get<NonManifoldSimplificationOptions>();

    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);
    Mesh& m = *mesh_in;

    auto pos_handle = m.get_attribute_handle<double>(options.position, wmtk::PrimitiveType::Vertex);

    if (pos_handle.mesh().top_simplex_type() != PrimitiveType::Triangle) {
        log_and_throw_error("This component was only tested for triangle meshes");
    }

    auto pass_through_attributes = base::get_attributes(cache, *mesh_in, options.pass_through);

    if (options.length_abs < 0) {
        if (options.length_rel < 0) {
            throw std::runtime_error("Either absolute or relative length must be set!");
        }
        options.length_abs = relative_to_absolute_length(pos_handle, options.length_rel);
    }

    // clear attributes
    {
        std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
        keeps.emplace_back(pos_handle);
        keeps.emplace_back(m.get_attribute_handle<int64_t>(
            options.non_manifold_vertex_label,
            PrimitiveType::Vertex));
        keeps.emplace_back(
            m.get_attribute_handle<int64_t>(options.non_manifold_edge_label, PrimitiveType::Edge));
        m.clear_attributes(keeps);
    }

    pos_handle = m.get_attribute_handle<double>(options.position, wmtk::PrimitiveType::Vertex);

    auto nmv_handle =
        m.get_attribute_handle<int64_t>(options.non_manifold_vertex_label, PrimitiveType::Vertex);
    auto nme_handle =
        m.get_attribute_handle<int64_t>(options.non_manifold_edge_label, PrimitiveType::Edge);
    auto nmv_acc = m.create_accessor<int64_t>(nmv_handle);
    auto nme_acc = m.create_accessor<int64_t>(nme_handle);

    auto do_not_collapse_handle = m.register_attribute<int64_t>(
        "non_manifold_simplification_do_not_collapse",
        PrimitiveType::Edge,
        1);
    auto do_not_collapse_acc = m.create_accessor<int64_t>(do_not_collapse_handle);

    // do not collapse any edge that is non-manifold or incident to something non-manifold
    for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
        if (nme_acc.const_scalar_attribute(t) == options.non_manifold_tag_value) {
            do_not_collapse_acc.scalar_attribute(t) = 1;
            continue;
        }

        const auto vs = simplex::faces_single_dimension_tuples(
            m,
            simplex::Simplex::edge(t),
            PrimitiveType::Vertex);

        assert(vs.size() == 2);

        if (nmv_acc.const_scalar_attribute(vs[0]) == options.non_manifold_tag_value ||
            nmv_acc.const_scalar_attribute(vs[1]) == options.non_manifold_tag_value) {
            do_not_collapse_acc.scalar_attribute(t) = 1;
        }
    }

    auto edge_length_attribute =
        m.register_attribute<double>("edge_length", PrimitiveType::Edge, 1);
    auto edge_length_accessor = m.create_accessor(edge_length_attribute.as<double>());
    // Edge length update
    auto compute_edge_length = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorXd::Constant(1, (P.col(0) - P.col(1)).norm());
    };
    auto edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            edge_length_attribute,
            pos_handle,
            compute_edge_length);
    edge_length_update->run_on_all();
    pass_through_attributes.emplace_back(edge_length_attribute);

    auto short_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({edge_length_accessor.scalar_attribute(s.tuple())});
    };

    //////////////////////////////////
    // envelope
    //////////////////////////////////
    auto envelope_invariant = std::make_shared<invariants::EnvelopeInvariant>(
        pos_handle,
        options.envelope_size,
        pos_handle);

    //////////////////////////////////
    // collapse transfer
    //////////////////////////////////
    auto clps_strat =
        std::make_shared<operations::CollapseNewAttributeStrategy<double>>(pos_handle);
    clps_strat->set_simplex_predicate(operations::BasicSimplexPredicate::IsInterior);
    clps_strat->set_strategy(operations::CollapseBasicStrategy::Default);

    //////////////////////////////////
    // Invariants
    //////////////////////////////////
    auto link_condition = std::make_shared<MultiMeshLinkConditionInvariant>(m);

    auto max_edge_length_invariant = std::make_shared<MaxEdgeLengthInvariant>(
        m,
        pos_handle.as<double>(),
        options.length_abs * options.length_abs);

    auto interior_edge_invariant = std::make_shared<InteriorEdgeInvariant>(m);
    auto interior_vertex_invariant = std::make_shared<InteriorVertexInvariant>(m);

    auto nme_invariant =
        std::make_shared<TodoInvariant>(m, do_not_collapse_handle.as<int64_t>(), 0);

    auto nmv_invariant = std::make_shared<TodoInvariant>(
        m,
        nmv_handle.as<int64_t>(),
        options.non_manifold_tag_value == 0 ? 1 : 0);

    //////////////////////////////////
    // EdgeCollapse
    //////////////////////////////////
    auto collapse = std::make_shared<operations::EdgeCollapse>(m);
    collapse->add_invariant(link_condition);

    collapse->set_new_attribute_strategy(pos_handle, clps_strat);
    collapse->set_new_attribute_strategy(nmv_handle, operations::CollapseBasicStrategy::None);
    collapse->set_new_attribute_strategy(nme_handle, operations::CollapseBasicStrategy::None);
    collapse->set_new_attribute_strategy(
        do_not_collapse_handle,
        operations::CollapseBasicStrategy::None);
    for (const auto& attr : pass_through_attributes) {
        collapse->set_new_attribute_strategy(attr);
    }

    collapse->add_transfer_strategy(edge_length_update);


    auto proj_collapse =
        std::make_shared<operations::composite::ProjectOperation>(collapse, pos_handle, pos_handle);

    proj_collapse->add_invariant(interior_edge_invariant);
    proj_collapse->add_invariant(nme_invariant);
    proj_collapse->add_invariant(max_edge_length_invariant);
    proj_collapse->add_invariant(envelope_invariant);

    proj_collapse->set_priority(short_edges_first);

    //////////////////////////////////
    // Smooth
    //////////////////////////////////
    auto smooth = std::make_shared<operations::AttributesUpdateWithFunction>(m);

    auto smoothing_function = [pos_handle](Mesh& m, const simplex::Simplex& v) -> bool {
        auto pos_acc = m.create_accessor<double>(pos_handle);

        const auto neighs = simplex::link_single_dimension(m, v, PrimitiveType::Vertex);

        pos_acc.vector_attribute(v.tuple()).setZero();
        for (const simplex::Simplex& n : neighs) {
            pos_acc.vector_attribute(v.tuple()) += pos_acc.const_vector_attribute(n.tuple());
        }
        pos_acc.vector_attribute(v.tuple()) /= neighs.size();

        return true;
    };

    smooth->set_function(smoothing_function);
    auto proj_smooth =
        std::make_shared<operations::composite::ProjectOperation>(smooth, pos_handle, pos_handle);

    proj_smooth->add_invariant(interior_vertex_invariant);
    proj_smooth->add_invariant(nmv_invariant);
    proj_smooth->add_invariant(envelope_invariant);


    write(m, options.output, 0, true);

    Scheduler scheduler;
    for (int64_t i = 0; i < options.iterations; ++i) {
        logger().info("Pass {}", i);
        {
            auto stats = scheduler.run_operation_on_all(*proj_collapse);
            logger().info(
                "Collapse {} ops (S/F) {}/{}. Execution time: {}",
                stats.number_of_performed_operations(),
                stats.number_of_successful_operations(),
                stats.number_of_failed_operations(),
                stats.executing_time);
        }
        {
            auto stats = scheduler.run_operation_on_all(*proj_smooth);
            logger().info(
                "Smooth {} ops (S/F) {}/{}. Execution time: {}",
                stats.number_of_performed_operations(),
                stats.number_of_successful_operations(),
                stats.number_of_failed_operations(),
                stats.executing_time);
        }

        m.consolidate();

        write(m, options.output, i + 1, true);
    }


    // clear attributes
    {
        std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
        keeps.emplace_back(pos_handle);
        keeps.emplace_back(nmv_handle);
        keeps.emplace_back(nme_handle);
        m.clear_attributes(keeps);
    }
}

} // namespace wmtk::components