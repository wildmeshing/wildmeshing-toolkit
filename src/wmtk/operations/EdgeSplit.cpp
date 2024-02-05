#include "EdgeSplit.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/tet_mesh/EdgeOperationData.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include <wmtk/utils/Logger.hpp>

#include "attribute_new/SplitNewAttributeStrategy.hpp"
#include "utils/multi_mesh_edge_split.hpp"

namespace wmtk::operations {

EdgeSplit::EdgeSplit(Mesh& m)
    : MeshOperation(m)
{
    auto collect_attrs = [&](auto&& mesh) {
        // can have const variant values here so gotta filter htose out
        if constexpr (!std::is_const_v<std::remove_reference_t<decltype(mesh)>>) {
            for (const auto& attr : mesh.custom_attributes()) {
                std::visit(
                    [&](auto&& tah) noexcept {
                        using T = typename std::decay_t<decltype(tah)>::Type;
                        m_new_attr_strategies.emplace_back(
                            std::make_shared<operations::SplitNewAttributeStrategy<T>>(
                                attribute::MeshAttributeHandle(mesh, attr)));
                    },
                    attr);
            }
        }
    };

    multimesh::MultiMeshVisitor custom_attribute_collector(collect_attrs);
    custom_attribute_collector.execute_from_root(m);
}

///////////////////////////////
std::vector<simplex::Simplex> EdgeSplit::execute_aux(
    EdgeMesh& mesh,
    const simplex::Simplex& simplex)
{
    auto return_data = utils::multi_mesh_edge_split(mesh, simplex.tuple(), m_new_attr_strategies);

    const edge_mesh::EdgeOperationData& my_data = return_data.get(mesh, simplex);

    return {simplex::Simplex::vertex(my_data.m_output_tuple)};
}

std::vector<simplex::Simplex> EdgeSplit::unmodified_primitives_aux(
    const EdgeMesh& mesh,
    const simplex::Simplex& simplex) const
{
    return {simplex};
}
///////////////////////////////


///////////////////////////////
std::vector<simplex::Simplex> EdgeSplit::execute_aux(TriMesh& mesh, const simplex::Simplex& simplex)
{
    auto return_data = utils::multi_mesh_edge_split(mesh, simplex.tuple(), m_new_attr_strategies);

    const tri_mesh::EdgeOperationData& my_data = return_data.get(mesh, simplex);

    return {simplex::Simplex::vertex(my_data.m_output_tuple)};
}

std::vector<simplex::Simplex> EdgeSplit::unmodified_primitives_aux(
    const TriMesh& mesh,
    const simplex::Simplex& simplex) const
{
    return {simplex};
}
///////////////////////////////


///////////////////////////////
std::vector<simplex::Simplex> EdgeSplit::execute_aux(TetMesh& mesh, const simplex::Simplex& simplex)
{
    auto return_data = utils::multi_mesh_edge_split(mesh, simplex.tuple(), m_new_attr_strategies);

    wmtk::logger().trace("{}", primitive_type_name(simplex.primitive_type()));

    const tet_mesh::EdgeOperationData& my_data = return_data.get(mesh, simplex);

    return {simplex::Simplex::vertex(my_data.m_output_tuple)};
}

std::vector<simplex::Simplex> EdgeSplit::unmodified_primitives_aux(
    const TetMesh& mesh,
    const simplex::Simplex& simplex) const
{
    return {simplex};
}
///////////////////////////////


std::shared_ptr<operations::BaseSplitNewAttributeStrategy> EdgeSplit::get_new_attribute_strategy(
    const attribute::MeshAttributeHandle& attribute) const
{
    for (auto& s : m_new_attr_strategies) {
        if (s->matches_attribute(attribute)) return s;
    }

    throw std::runtime_error("unable to find attribute");
}

void EdgeSplit::set_new_attribute_strategy(
    const attribute::MeshAttributeHandle& attribute,
    const std::shared_ptr<operations::BaseSplitNewAttributeStrategy>& other)
{
    for (size_t i = 0; i < m_new_attr_strategies.size(); ++i) {
        if (m_new_attr_strategies[i]->matches_attribute(attribute)) {
            m_new_attr_strategies[i] = other;
            return;
        }
    }

    throw std::runtime_error("unable to find attribute");
}

void EdgeSplit::set_new_attribute_strategy(
    const attribute::MeshAttributeHandle& attribute,
    const wmtk::operations::SplitBasicStrategy& spine,
    const wmtk::operations::SplitRibBasicStrategy& rib)
{
    std::visit(
        [&](auto&& val) -> void {
            using T = typename std::decay_t<decltype(val)>::Type;
            using OpType = operations::SplitNewAttributeStrategy<T>;

            std::shared_ptr<OpType> tmp = std::make_shared<OpType>(attribute);
            tmp->set_strategy(spine);
            tmp->set_rib_strategy(rib);

            set_new_attribute_strategy(attribute, tmp);
        },
        attribute.handle());
}

std::pair<Tuple, Tuple> EdgeSplit::new_spine_edges(const Mesh& mesh, const Tuple& new_vertex)
{
    // new_vertex is a spine edge on a face pointing to the new vertex, so we
    // * PE -> new edge
    // * PF -> other face
    // * PE -> other spine edge
    constexpr static PrimitiveType PE = PrimitiveType::Edge;
    constexpr static PrimitiveType PF = PrimitiveType::Triangle;
    constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

    std::pair<Tuple, Tuple> ret;

    switch (mesh.top_simplex_type()) {
    case PE: {
        ret = {new_vertex, mesh.switch_tuples(new_vertex, {PE})};
        break;
    }
    case PF: {
        ret = {new_vertex, mesh.switch_tuples(new_vertex, {PE, PF, PE})};
        break;
    }
    case PT: {
        ret = {new_vertex, mesh.switch_tuples(new_vertex, {PE, PF, PT, PF, PE})};
        break;
    }
    case PrimitiveType::Vertex:
    default: assert(false); // "Invalid top simplex"
    }
    return ret;
}


} // namespace wmtk::operations
