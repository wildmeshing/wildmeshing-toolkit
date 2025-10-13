#include "isotropic_remeshing_regulated.hpp"

#include "IsotropicRemeshingOptions.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/invariants/FusionEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/invariants/MaxEdgeLengthInvariant.hpp>
#include <wmtk/invariants/MinEdgeLengthInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/MultiMeshMapValidInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/ValenceImprovementInvariant.hpp>
#include <wmtk/invariants/uvEdgeInvariant.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/multimesh/consolidate.hpp>
#include <wmtk/operations/AttributesUpdate.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/MeshConsolidate.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include <wmtk/operations/utils/VertexLaplacianSmooth.hpp>
#include <wmtk/operations/utils/VertexTangentialLaplacianSmooth.hpp>
#include <wmtk/simplex/RawSimplex.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/random_seed.hpp>

#include <polysolve/Utils.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <random>
#include <map>
#include <set>
#include <vector>
#include <vector>

namespace wmtk::components::isotropic_remeshing {

namespace {

simplex::RawSimplex vertex_key(const TriMesh& mesh, const Tuple& tuple)
{
    return simplex::RawSimplex(mesh, simplex::Simplex::vertex(mesh, tuple));
}

std::vector<simplex::RawSimplex> one_ring_vertex_keys(const TriMesh& mesh, const Tuple& vertex_tuple)
{
    std::vector<simplex::RawSimplex> keys;
    const auto ring =
        simplex::link(mesh, simplex::Simplex::vertex(mesh, vertex_tuple)).simplex_vector(
            PrimitiveType::Vertex);
    keys.reserve(ring.size());
    for (const auto& v : ring) {
        keys.emplace_back(mesh, v);
    }
    return keys;
}

int64_t predicted_post_collapse_valence(const TriMesh& mesh, const Tuple& v0, const Tuple& v1)
{
    std::set<simplex::RawSimplex> neighbor_keys;
    const simplex::RawSimplex key0 = vertex_key(mesh, v0);
    const simplex::RawSimplex key1 = vertex_key(mesh, v1);

    auto accumulate_ring = [&](const Tuple& center, const simplex::RawSimplex& opposite_key) {
        const simplex::RawSimplex center_key = vertex_key(mesh, center);
        const auto ring =
            simplex::link(mesh, simplex::Simplex::vertex(mesh, center)).simplex_vector(
                PrimitiveType::Vertex);
        for (const auto& n : ring) {
            simplex::RawSimplex key(mesh, n);
            if (key == opposite_key || key == center_key) {
                continue;
            }
            neighbor_keys.insert(key);
        }
    };

    accumulate_ring(v0, key1);
    accumulate_ring(v1, key0);

    return static_cast<int64_t>(neighbor_keys.size());
}

SchedulerStats run_regulated_edge_operation(
    operations::Operation& op,
    const RegulatedIsotropicRemeshingOptions& options)
{
    SchedulerStats res;
    std::vector<simplex::Simplex> simplices;
    std::vector<std::pair<int64_t, double>> order;

    const auto type = op.primitive_type();
    assert(type == PrimitiveType::Edge);
    auto& mesh = static_cast<TriMesh&>(op.mesh());

    {
        POLYSOLVE_SCOPED_STOPWATCH("Collecting primitives", res.collecting_time, logger());
        const auto tups = mesh.get_all(type);
        simplices = wmtk::simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
            mesh,
            tups,
            type);
    }

    {
        POLYSOLVE_SCOPED_STOPWATCH("Sorting", res.sorting_time, logger());
        if (op.use_random_priority()) {
            std::mt19937 gen(utils::get_random_seed());
            std::shuffle(simplices.begin(), simplices.end(), gen);
        } else {
            order.reserve(simplices.size());
            for (int64_t i = 0; i < simplices.size(); ++i) {
                order.emplace_back(i, op.priority(simplices[i]));
            }

            std::stable_sort(order.begin(), order.end(), [](const auto& a, const auto& b) {
                return a.second < b.second;
            });
        }
    }

    std::map<simplex::RawSimplex, int64_t> vertex_collapse_count;
    std::set<simplex::RawSimplex> blocked_vertices;

    auto is_vertex_blocked = [&](const Tuple& v) {
        const simplex::RawSimplex key = vertex_key(mesh, v);
        if (blocked_vertices.count(key) > 0) {
            return true;
        }
        if (options.max_collapses_per_vertex > 0) {
            const auto it = vertex_collapse_count.find(key);
            if (it != vertex_collapse_count.end() &&
                it->second >= options.max_collapses_per_vertex) {
                return true;
            }
        }
        return false;
    };

    auto attempt_simplex = [&](const simplex::Simplex& simplex) {
        const Tuple& edge_tuple = simplex.tuple();
        const Tuple v0 = edge_tuple;
        const Tuple v1 = mesh.switch_tuple(edge_tuple, PrimitiveType::Vertex);

        if (is_vertex_blocked(v0) || is_vertex_blocked(v1)) {
            if (options.count_skipped_as_failures) {
                res.fail();
            }
            return;
        }

        if (options.enforce_max_valence) {
            const int64_t predicted_valence = predicted_post_collapse_valence(mesh, v0, v1);
            if (predicted_valence > options.max_post_collapse_valence) {
                if (options.count_skipped_as_failures) {
                    res.fail();
                }
                return;
            }
        }

        std::vector<simplex::RawSimplex> one_ring_block_keys;
        if (options.limit_one_ring_collapses) {
            one_ring_block_keys = one_ring_vertex_keys(mesh, v0);
            const auto from_v1 = one_ring_vertex_keys(mesh, v1);
            one_ring_block_keys.insert(
                one_ring_block_keys.end(),
                from_v1.begin(),
                from_v1.end());
        }

        auto mods = op(simplex);
        if (mods.empty()) {
            res.fail();
            return;
        }

        res.succeed();

        const simplex::RawSimplex key0 = vertex_key(mesh, v0);
        const simplex::RawSimplex key1 = vertex_key(mesh, v1);
        auto& count0 = vertex_collapse_count[key0];
        ++count0;
        auto& count1 = vertex_collapse_count[key1];
        ++count1;

        if (options.limit_one_ring_collapses) {
            blocked_vertices.insert(one_ring_block_keys.begin(), one_ring_block_keys.end());
            blocked_vertices.insert(key0);
            blocked_vertices.insert(key1);
        }

        if (options.max_collapses_per_vertex > 0 && count0 >= options.max_collapses_per_vertex) {
            blocked_vertices.insert(key0);
        }
        if (options.max_collapses_per_vertex > 0 && count1 >= options.max_collapses_per_vertex) {
            blocked_vertices.insert(key1);
        }
    };

    {
        POLYSOLVE_SCOPED_STOPWATCH("Executing operation", res.executing_time, logger());

        if (op.use_random_priority()) {
            for (const auto& simplex : simplices) {
                attempt_simplex(simplex);
            }
        } else {
            for (const auto& entry : order) {
                attempt_simplex(simplices[entry.first]);
            }
        }
    }

    return res;
}

double relative_to_absolute_length(
    const attribute::MeshAttributeHandle& position,
    const double length_rel)
{
    auto pos = position.mesh().create_const_accessor<double>(position);
    const auto vertices = position.mesh().get_all(PrimitiveType::Vertex);
    Eigen::AlignedBox<double, Eigen::Dynamic> bbox(pos.dimension());

    for (const auto& v : vertices) {
        bbox.extend(pos.const_vector_attribute(v));
    }

    const double diag_length = bbox.sizes().norm();

    return length_rel * diag_length;
}

} // namespace

void isotropic_remeshing_regulated(const RegulatedIsotropicRemeshingOptions& options)
{
    auto position = options.position_attribute;

    if (position.mesh().top_simplex_type() != PrimitiveType::Triangle) {
        log_and_throw_error(
            "isotropic remeshing works only for triangle meshes: {}",
            primitive_type_name(position.mesh().top_simplex_type()));
    }

    auto pass_through_attributes = options.pass_through_attributes;
    auto other_positions = options.other_position_attributes;

    double length = options.length_abs;
    if (options.length_abs < 0) {
        if (options.length_rel < 0) {
            throw std::runtime_error("Either absolute or relative length must be set!");
        }
        length = relative_to_absolute_length(position, options.length_rel);
    }

    std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
    keeps.emplace_back(position);
    keeps.insert(keeps.end(), other_positions.begin(), other_positions.end());

    std::optional<attribute::MeshAttributeHandle> position_for_inversion =
        options.inversion_position_attribute;

    assert(dynamic_cast<TriMesh*>(&position.mesh()) != nullptr);

    TriMesh& mesh = static_cast<TriMesh&>(position.mesh());

    const double length_min = (4. / 5.) * length;
    const double length_max = (4. / 3.) * length;

    std::vector<attribute::MeshAttributeHandle> positions = other_positions;
    positions.push_back(position);

    auto invariant_link_condition =
        std::make_shared<wmtk::invariants::MultiMeshLinkConditionInvariant>(mesh);

    auto invariant_min_edge_length = std::make_shared<MinEdgeLengthInvariant>(
        mesh,
        position.as<double>(),
        length_max * length_max);

    auto invariant_max_edge_length = std::make_shared<MaxEdgeLengthInvariant>(
        mesh,
        position.as<double>(),
        length_min * length_min);

    auto invariant_interior_edge = std::make_shared<invariants::InvariantCollection>(mesh);
    auto invariant_interior_vertex = std::make_shared<invariants::InvariantCollection>(mesh);

    auto set_all_invariants = [&](auto&& m) {
        invariant_interior_edge->add(
            std::make_shared<invariants::InteriorSimplexInvariant>(m, PrimitiveType::Edge));
        invariant_interior_vertex->add(
            std::make_shared<invariants::InteriorSimplexInvariant>(m, PrimitiveType::Vertex));
    };
    multimesh::MultiMeshVisitor visitor(set_all_invariants);
    visitor.execute_from_root(mesh);

    auto invariant_valence_improve =
        std::make_shared<invariants::ValenceImprovementInvariant>(mesh);

    auto invariant_mm_map = std::make_shared<MultiMeshMapValidInvariant>(mesh);

    auto update_position_func = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        return P.col(0);
    };
    std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>
        update_position;

    if (!options.other_position_attributes.empty()) {
        update_position =
            std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
                other_positions.front(),
                position,
                update_position_func);
    }

    using namespace operations;

    assert(mesh.is_connectivity_valid());

    // split
    wmtk::logger().debug("Configure regulated isotropic remeshing split");
    auto op_split = std::make_shared<EdgeSplit>(mesh);
    op_split->add_invariant(invariant_min_edge_length);
    if (options.lock_boundary && !options.use_for_periodic && !options.dont_disable_split) {
        op_split->add_invariant(invariant_interior_edge);
    }
    for (auto& p : positions) {
        op_split->set_new_attribute_strategy(
            p,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
    }
    for (const auto& attr : pass_through_attributes) {
        op_split->set_new_attribute_strategy(attr);
    }
    assert(op_split->attribute_new_all_configured());

    // collapse
    wmtk::logger().debug("Configure regulated isotropic remeshing collapse");
    auto op_collapse = std::make_shared<EdgeCollapse>(mesh);
    op_collapse->add_invariant(invariant_link_condition);
    if (position_for_inversion) {
        op_collapse->add_invariant(std::make_shared<SimplexInversionInvariant<double>>(
            position_for_inversion.value().mesh(),
            position_for_inversion.value().as<double>()));
    }

    op_collapse->add_invariant(invariant_max_edge_length);
    op_collapse->add_invariant(invariant_mm_map);

    if (options.fix_uv_seam) {
        op_collapse->add_invariant(
            std::make_shared<invariants::uvEdgeInvariant>(mesh, other_positions.front().mesh()));
    }

    if (options.lock_boundary && !options.use_for_periodic) {
        op_collapse->add_invariant(invariant_interior_edge);
        for (auto& p : positions) {
            auto tmp = std::make_shared<CollapseNewAttributeStrategy<double>>(p);
            tmp->set_strategy(CollapseBasicStrategy::Mean);
            tmp->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
            op_collapse->set_new_attribute_strategy(p, tmp);
        }
    } else if (options.use_for_periodic) {
        op_collapse->add_invariant(
            std::make_shared<invariants::FusionEdgeInvariant>(mesh, mesh.get_multi_mesh_root()));
        for (auto& p : positions) {
            op_collapse->set_new_attribute_strategy(p, CollapseBasicStrategy::Mean);
        }
    } else {
        for (auto& p : positions) {
            op_collapse->set_new_attribute_strategy(p, CollapseBasicStrategy::Mean);
        }
    }

    for (const auto& attr : pass_through_attributes) {
        op_collapse->set_new_attribute_strategy(attr);
    }
    assert(op_collapse->attribute_new_all_configured());

    // swap
    wmtk::logger().debug("Configure regulated isotropic remeshing swap");
    auto op_swap = std::make_shared<composite::TriEdgeSwap>(mesh);
    op_swap->add_invariant(invariant_interior_edge);

    if (options.fix_uv_seam) {
        op_swap->add_invariant(
            std::make_shared<invariants::uvEdgeInvariant>(mesh, other_positions.front().mesh()));
    }

    op_swap->add_invariant(invariant_valence_improve);
    op_swap->collapse().add_invariant(invariant_link_condition);
    op_swap->collapse().add_invariant(invariant_mm_map);
    for (auto& p : positions) {
        op_swap->split().set_new_attribute_strategy(
            p,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
    }
    if (position_for_inversion) {
        op_swap->collapse().add_invariant(std::make_shared<SimplexInversionInvariant<double>>(
            position_for_inversion.value().mesh(),
            position_for_inversion.value().as<double>()));
    }

    for (auto& p : positions) {
        op_swap->collapse().set_new_attribute_strategy(p, CollapseBasicStrategy::CopyOther);
    }
    for (const auto& attr : pass_through_attributes) {
        op_swap->split().set_new_attribute_strategy(attr);
        op_swap->collapse().set_new_attribute_strategy(attr);
    }
    assert(op_swap->split().attribute_new_all_configured());
    assert(op_swap->collapse().attribute_new_all_configured());

    // smooth
    auto op_smooth = std::make_shared<AttributesUpdateWithFunction>(mesh);
    if (position.dimension() == 3) {
        op_smooth->set_function(VertexTangentialLaplacianSmooth(position));
    } else {
        op_smooth->set_function(VertexLaplacianSmooth(position));
    }

    if (options.lock_boundary) {
        op_smooth->add_invariant(invariant_interior_vertex);
    }

    if (options.fix_uv_seam) {
        op_smooth->add_invariant(
            std::make_shared<invariants::uvEdgeInvariant>(mesh, other_positions.front().mesh()));
    }

    if (position_for_inversion) {
        op_smooth->add_invariant(std::make_shared<SimplexInversionInvariant<double>>(
            position_for_inversion.value().mesh(),
            position_for_inversion.value().as<double>()));
    }

    if (update_position) {
        op_smooth->add_transfer_strategy(update_position);
    }

    Scheduler scheduler;
    for (long i = 0; i < options.iterations; ++i) {
        wmtk::logger().info("Regulated isotropic iteration {}", i);

        SchedulerStats pass_stats;

        pass_stats += scheduler.run_operation_on_all(*op_split);
        pass_stats += run_regulated_edge_operation(*op_collapse, options);
        pass_stats += scheduler.run_operation_on_all(*op_swap);
        pass_stats += scheduler.run_operation_on_all(*op_smooth);

        auto op_consolidate = MeshConsolidate(mesh);
        op_consolidate(simplex::Simplex(mesh, PrimitiveType::Vertex, Tuple()));

        logger().info(
            "Executed {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, executing: {}",
            pass_stats.number_of_performed_operations(),
            pass_stats.number_of_successful_operations(),
            pass_stats.number_of_failed_operations(),
            pass_stats.collecting_time,
            pass_stats.sorting_time,
            pass_stats.executing_time);
    }
}

} // namespace wmtk::components::isotropic_remeshing
