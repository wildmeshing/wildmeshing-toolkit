#pragma once

#include "MultiMeshSimplexVisitor.hpp"

namespace wmtk::multimesh {
template <int64_t cell_dimension, typename Functor>
class MultiMeshSimplexEventVisitor
{
public:
    using VisitorType = MultiMeshSimplexVisitor<cell_dimension, Functor>;
    MultiMeshSimplexEventVisitor(const MultiMeshSimplexVisitor<cell_dimension, Functor>& visitor)
        : m_visitor(visitor)
    {}
    template <typename T>
    using GetReturnType_t = typename VisitorType::template GetReturnType_t<T>;

    auto node_events() const { return m_visitor.node_events(); }
    const auto& edge_events() const { return m_visitor.edge_events(); }

    template <typename NodeFunctor>
    void run_on_nodes(NodeFunctor&& node_functor);
    template <typename EdgeFunctor>
    void run_on_edges(EdgeFunctor&& edge_functor);

    template <typename MeshType, typename... OtherArgumentTypes>
    const auto& get_cached_return(const MeshType& m, OtherArgumentTypes&&... args) const
    {
        return m_visitor.cache().get(m, std::forward<OtherArgumentTypes>(args)...);
    }

private:
    const VisitorType& m_visitor;
};
template <int64_t cell_dimension, typename Functor>
MultiMeshSimplexEventVisitor(const MultiMeshSimplexVisitor<cell_dimension, Functor>& visitor)
    -> MultiMeshSimplexEventVisitor<cell_dimension, Functor>;

template <int64_t cell_dimension, typename Functor>
template <typename EdgeFunctor>
void MultiMeshSimplexEventVisitor<cell_dimension, Functor>::run_on_edges(EdgeFunctor&& edge_functor)
{
    // go through every edge event and run the edge functor on it
    for (const auto& pr : edge_events()) {
        // why does clang hate structured bindings so much?
        const auto& keyA = std::get<0>(pr);
        const auto& keyB = std::get<1>(pr);
        // const auto& [parent_ptr, sa] = keyA;
        // const auto& [child_ptr, sb] = keyB;


        // extract all the specific types for the edge events
        std::visit(
            [&](auto parent_mesh_, auto child_mesh_) noexcept {
                auto& parent_mesh = parent_mesh_.get();
                auto& child_mesh = child_mesh_.get();
                using ChildType = std::decay_t<decltype(child_mesh)>;
                using ParentType = std::decay_t<decltype(parent_mesh)>;

                // logger().trace(
                //     "going through edge cache {} => {}",
                //     fmt::join(parent_mesh.absolute_multi_mesh_id(), ","),
                //     fmt::join(child_mesh.absolute_multi_mesh_id(), ","));

                constexpr static int64_t ParentDim =
                    wmtk::utils::metaprogramming::cell_dimension_v<ParentType>;
                constexpr static int64_t ChildDim =
                    wmtk::utils::metaprogramming::cell_dimension_v<ChildType>;

                using ParentReturnType = GetReturnType_t<ParentType>;
                using ChildReturnType = GetReturnType_t<ChildType>;

                constexpr static bool ChildHasReturn = !std::is_void_v<ChildReturnType>;
                constexpr static bool ParentHasReturn = !std::is_void_v<ParentReturnType>;

                if constexpr (ParentDim >= ChildDim && ChildHasReturn && ParentHasReturn) {
                    const ParentReturnType& parent_return =
                        get_cached_return(parent_mesh, std::get<1>(keyA));
                    const ChildReturnType& child_return =
                        get_cached_return(child_mesh, std::get<1>(keyB));
                    edge_functor(
                        parent_mesh,
                        std::get<1>(keyA),
                        parent_return,
                        child_mesh,
                        std::get<1>(keyB),
                        child_return);
                }
            },
            // TODO: this const casting is ugly, const referencing for the edge functor needs to
            // be fixed
            wmtk::utils::metaprogramming::as_mesh_variant(*const_cast<Mesh*>(std::get<0>(keyA))),
            wmtk::utils::metaprogramming::as_mesh_variant(*const_cast<Mesh*>(std::get<0>(keyB))));
    }
}

template <int64_t cell_dimension, typename Functor>
template <typename NodeFunctor>
void MultiMeshSimplexEventVisitor<cell_dimension, Functor>::run_on_nodes(NodeFunctor&& node_functor)
{
    // go through every edge event and run the edge functor on it
    for (const auto& event : node_events()) {
        // extract all the specific types for the edge events
        std::visit(
            [&](auto parent_mesh_) noexcept {
                auto& parent_mesh = parent_mesh_.get();
                using ParentType = std::decay_t<decltype(parent_mesh)>;


                constexpr static int64_t ParentDim =
                    wmtk::utils::metaprogramming::cell_dimension_v<ParentType>;

                using ParentReturnType = GetReturnType_t<ParentType>;

                constexpr static bool ParentHasReturn = !std::is_void_v<ParentReturnType>;

                if constexpr (ParentHasReturn) {
                    const ParentReturnType& parent_return =
                        get_cached_return(parent_mesh, std::get<1>(event));
                    node_functor(parent_mesh, std::get<1>(event), parent_return);
                }
            },
            // TODO: this const casting is ugly, const referencing for the edge functor needs to
            // be fixed
            wmtk::utils::metaprogramming::as_mesh_variant(*const_cast<Mesh*>(std::get<0>(event))));
    }
}

} // namespace wmtk::multimesh
