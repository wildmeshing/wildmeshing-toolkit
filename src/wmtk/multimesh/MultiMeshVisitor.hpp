#pragma once
#include <type_traits>
#include <variant> //to get monostage
#include <wmtk/Mesh.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/operations/tri_mesh/EdgeOperationData.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/mesh_type_from_primitive_type.hpp>
#include <wmtk/utils/metaprogramming/MeshVariantTraits.hpp>
#include <wmtk/utils/metaprogramming/ReferenceWrappedFunctorReturnCache.hpp>
#include <wmtk/utils/metaprogramming/ReferenceWrappedFunctorReturnType.hpp>
#include <wmtk/utils/metaprogramming/as_mesh_variant.hpp>
#include <wmtk/utils/metaprogramming/cell_dimension.hpp>
#include "utils/CachedMeshVariantReturnValues.hpp"

// TODO: extend this visitor to support const and non-const references
#define WMTK_MESH_VISITOR_ONLY_SUPPORTS_NONCONST_REFERENCE

namespace wmtk::multimesh {

// if NodeFunctor returns a value then
template <typename MMVisitor>
class MultiMeshVisitorExecutor;

template <long cell_dimension_, typename NodeFunctor_, typename EdgeFunctor_ = std::monostate>
class MultiMeshVisitor
{
public:
    using MeshVariantTraits = wmtk::utils::metaprogramming::MeshVariantTraits;
    using NodeFunctor = NodeFunctor_;
    using EdgeFunctor = EdgeFunctor_;
    constexpr static bool HasEdgeFunctor = !std::is_same_v<EdgeFunctor, std::monostate>;
    constexpr static long cell_dimension = cell_dimension_;

    using CacheType = wmtk::utils::metaprogramming::
        ReferenceWrappedFunctorReturnCache<NodeFunctor, MeshVariantTraits, Simplex>;

    /* @brief constructor that takes in the node and edg efunctors
     *
     * @param f The functor that will be run on each mesh in the tree
     * @param ef The functor that will be run on each (mesh,result),(mesh,result) pair after running on the nodes
     * */
    MultiMeshVisitor(NodeFunctor&& f, EdgeFunctor&& ef)
        : m_node_functor(f)
        , m_edge_functor(ef)
    {}

    /* @brief constructor that takes in the node and edg efunctors
     *
     * @param f The functor that will be run on each mesh in the tree
     * */
    MultiMeshVisitor(NodeFunctor&& f)
        : m_node_functor(f)
    {
        // this constructor is disallowed if there is an edge functor
        assert(std::is_same_v<EdgeFunctor_, std::monostate>);
    }

    /* @brief utility constructor that delegates a constant for class template arugment deduciton
     * @param _ deduction hint that helps pick cell_dimension
     * @param f The functor that will be run on each mesh in the tree
     * @param ef The functor that will be run on each (mesh,result),(mesh,result) pair after running on the nodes
     * */
    MultiMeshVisitor(
        std::integral_constant<long, cell_dimension>,
        NodeFunctor&& f,
        EdgeFunctor&& ef)
        : MultiMeshVisitor(std::forward<NodeFunctor>(f), std::forward<EdgeFunctor>(ef))
    {}
    /* @brief utility constructor that delegates a constant for class template arugment deduciton
     * @param _ deduction hint that helps pick cell_dimension
     * @param f The functor that will be run on each mesh in the tree
     * */
    MultiMeshVisitor(std::integral_constant<long, cell_dimension>, NodeFunctor&& f)
        : MultiMeshVisitor(std::forward<NodeFunctor>(f))
    {}


    template <typename MMVisitor_>
    friend class MultiMeshVisitorExecutor;
    using Executor =
        MultiMeshVisitorExecutor<MultiMeshVisitor<cell_dimension, NodeFunctor, EdgeFunctor>>;

    /* @brief executes the node functor (and potentially edge functor) from the entire graph
     * @param mesh some mesh in the tree that the operation needs to run from
     * @param simplex the simplex on the input mesh that we want to run the operation from
     * @return a ReferenceWrappedFunctorReturnCache that lets one request (mesh,simplex) -> NodeFunctor Return type
     * */
    // even if you try to use an interior mesh node this always just uses the root
    auto execute_from_root(Mesh& mesh, const simplex::Simplex& simplex) const
    {
        // if the user passed in a mesh class lets try re-invoking with a derived type
        Mesh& root_base_mesh = mesh.get_multi_mesh_root();
        auto mesh_root_variant = wmtk::utils::metaprogramming::as_mesh_variant(root_base_mesh);
        const simplex::Simplex root_simplex = mesh.map_to_root(simplex);
        assert(root_base_mesh.is_valid_slow(root_simplex.tuple()));
        Executor exec(*this);
        std::visit([&](auto&& root) { exec.execute(root.get(), root_simplex); }, mesh_root_variant);
        if constexpr (!std::is_same_v<CacheType, std::monostate>) {
            return std::move(exec.m_return_data);
        }
    }
    /* @brief executes the node functor (and potentially edge functor) from the subtree of the input node
     * @param mesh the mesh in the tree that the operation needs to run from
     * @param simplex the simplex on the input mesh that we want to run the operation from
     * @return a ReferenceWrappedFunctorReturnCache that lets one request (mesh,simplex) -> NodeFunctor Return type
     * */
    template <typename MeshType>
    auto&& execute_mesh(MeshType&& mesh, const simplex::Simplex& simplex) const
    {
        static_assert(
            !std::is_same_v<std::decay_t<MeshType>, Mesh>,
            "Don't pass in a mesh, use variant/visitor to get its derived type");
        Executor exec(*this);
        exec.execute(std::forward<MeshType>(mesh), simplex);
        if constexpr (!std::is_same_v<CacheType, std::monostate>) {
            return std::move(exec.m_return_data);
        }
    }


protected:
    NodeFunctor m_node_functor;
    EdgeFunctor m_edge_functor;
};

// if NodeFunctor returns a value then
template <typename MMVisitor>
class MultiMeshVisitorExecutor
{
public:
    constexpr static long cell_dimension = MMVisitor::cell_dimension;
    using MeshVariantTraits = wmtk::utils::metaprogramming::MeshVariantTraits;
    using NodeFunctor = typename MMVisitor::NodeFunctor;
    using EdgeFunctor = typename MMVisitor::EdgeFunctor;
    using TypeHelper = wmtk::utils::metaprogramming::detail::ReferenceWrappedFunctorReturnType<
        NodeFunctor,
        MeshVariantTraits::AllReferenceTuple,
        wmtk::simplex::Simplex>;
    // template <bool IsConst, typename MeshType>
#if defined(WMTK_MESH_VISITOR_ONLY_SUPPORTS_NONCONST_REFERENCE)
    template <typename MeshType>
    using GetReturnType_t = typename TypeHelper::template ReturnType<MeshType>;
    template <typename MeshType>
    constexpr static bool HasReturnValue_v = !std::is_void_v<GetReturnType_t<MeshType>>;
#else
    template <bool IsConst, typename MeshType>
    using GetReturnType_t = typename TypeHelper::template ReturnType<IsConst, MeshType>;
    template <bool IsConst, typename MeshType>
    constexpr static bool HasReturnValue_v = !std::is_void_v<GetReturnType_t<IsConst, MeshType>>;
#endif

    using ReturnDataType = wmtk::utils::metaprogramming::
        ReferenceWrappedFunctorReturnCache<NodeFunctor, MeshVariantTraits, Simplex>;
    constexpr static bool HasReturnCache =
        !wmtk::utils::metaprogramming::all_return_void_v<NodeFunctor, MeshVariantTraits, Simplex>;

    MultiMeshVisitorExecutor(const MMVisitor& v)
        : visitor(v)
    {}

    ReturnDataType m_return_data;
    const MMVisitor& visitor;


    constexpr static bool HasEdgeFunctor = MMVisitor::HasEdgeFunctor;


    /* @brief runs the node functor on every node in the subgraph and then runs hte edge functor if it exists
     * @param mesh the mesh whose subgraph will be run
     * @param simplex the simplex whose subgraph will be run
     * */
    template <typename MeshType>
    void execute(MeshType&& mesh, const simplex::Simplex& simplex)
    {
        static_assert(std::is_base_of_v<Mesh, std::decay_t<MeshType>>);
        run(std::forward<MeshType>(mesh), simplex);

        if constexpr (HasReturnCache && HasEdgeFunctor) {
            // go through every edge event and run the edge functor on it
            for (const auto& pr : edge_events) {

                // why does clang hate structured bindings so much?
                const auto& keyA = std::get<0>(pr);
                const auto& keyB = std::get<1>(pr);
                //const auto& [parent_ptr, sa] = keyA;
                //const auto& [child_ptr, sb] = keyB;
                

                // extract all the specific types for the edge events
                std::visit(
                    [&](auto parent_mesh_, auto child_mesh_) noexcept {
                        auto& parent_mesh = parent_mesh_.get();
                        auto& child_mesh = child_mesh_.get();
                        using ChildType = std::decay_t<decltype(child_mesh)>;
                        using ParentType = std::decay_t<decltype(parent_mesh)>;

                        // spdlog::warn(
                        //     "going through edge cache {} => {}",
                        //     fmt::join(parent_mesh.absolute_multi_mesh_id(), ","),
                        //     fmt::join(child_mesh.absolute_multi_mesh_id(), ","));

                        constexpr static long ParentDim =
                            wmtk::utils::metaprogramming::cell_dimension_v<ParentType>;
                        constexpr static long ChildDim =
                            wmtk::utils::metaprogramming::cell_dimension_v<ChildType>;

                        using ParentReturnType = GetReturnType_t<ParentType>;
                        using ChildReturnType = GetReturnType_t<ChildType>;

                        constexpr static bool ChildHasReturn = !std::is_void_v<ChildReturnType>;
                        constexpr static bool ParentHasReturn = !std::is_void_v<ParentReturnType>;

                        if constexpr (ParentDim >= ChildDim && ChildHasReturn && ParentHasReturn) {
                            const ParentReturnType& parent_return =
                                m_return_data.get(parent_mesh, std::get<1>(keyA));
                            const ChildReturnType& child_return = m_return_data.get(child_mesh, std::get<1>(keyB));
                            // spdlog::warn(
                            //     "MultiMeshVisitor[{}=>{}] adding to edges edge simplex {} "
                            //     "child "
                            //     "simplex{}",
                            //     fmt::join(parent_mesh.absolute_multi_mesh_id(), ","),
                            //     fmt::join(child_mesh.absolute_multi_mesh_id(), ","),
                            //     wmtk::utils::TupleInspector::as_string(sa.tuple()),
                            //     wmtk::utils::TupleInspector::as_string(sb.tuple()));
                            

                            // with all of the event types properly declared / prepared for, actually run the edge functor
                            visitor.m_edge_functor(
                                parent_mesh,
                                parent_return,
                                child_mesh,
                                child_return);

                            // fully generic visitor pattern if we ever need it / for debugging
                            /*
                            const auto& parent_return = m_return_data.get_variant(keyA);
                            const auto& child_return = m_return_data.get_variant(keyB);
                            std::visit(
                                [&](const auto& pr, const auto& cr) noexcept {
                                    if constexpr (
                                        ParentDim >= ChildDim && ChildHasReturn &&
                                        ParentHasReturn &&
                                        std::is_same_v<
                                            std::decay_t<decltype(pr)>,
                                            std::decay_t<ParentReturnType>> &&
                                        std::is_same_v<
                                            std::decay_t<decltype(cr)>,
                                            std::decay_t<ChildReturnType>>) {
                                        visitor.m_edge_functor(parent_mesh, pr, child_mesh, cr);
                                    }
                                },
                                parent_return,
                                child_return);
                                */
                        }
                    },
                    // TODO: this const casting is ugly, const referencing for the edge functor needs to be fixed
                    wmtk::utils::metaprogramming::as_mesh_variant(*const_cast<Mesh*>(std::get<0>(keyA))),
                    wmtk::utils::metaprogramming::as_mesh_variant(*const_cast<Mesh*>(std::get<0>(keyB))));
            }
        }
    }


private:

    /* @brief runs the node functor on every node in the subgraph
     * @param mesh the mesh whose subgraph will be run
     * @param simplex the simplex whose subgraph will be run
     * */
    template <typename MeshType_>
    void run(MeshType_&& current_mesh, const simplex::Simplex& simplex)
    {
        assert(current_mesh.is_valid_slow(simplex.tuple()));
        using MeshType = std::decay_t<MeshType_>;


        // short circuit operations that happen below the desired dimension
        constexpr static long MeshDim = wmtk::utils::metaprogramming::cell_dimension_v<MeshType>;
        if constexpr (cell_dimension > MeshDim) {
            return;
        }
#if defined(WMTK_MESH_VISITOR_ONLY_SUPPORTS_NONCONST_REFERENCE)
        using CurReturnType = GetReturnType_t<MeshType>;
#else
        constexpr static bool CurIsConst = std::is_const_v<MeshType>;
        using CurReturnType = GetReturnType_t<CurIsConst, MeshType>;
#endif

        constexpr static bool CurHasReturn = !std::is_void_v<CurReturnType>;

        // pre-compute all of  the child tuples in case the node functor changes the mesh that
        // breaks the traversal down
        auto& child_datas = current_mesh.m_multi_mesh_manager.children();
        std::vector<std::vector<Simplex>> mapped_child_simplices;
        mapped_child_simplices.reserve(child_datas.size());


        // in-place convert this tuple into all of the child simplices
        // TODO: this repeatedly extracts every version of hte input smiplex,
        // we could cache this in the future
        std::transform(
            child_datas.begin(),
            child_datas.end(),
            std::back_inserter(mapped_child_simplices),
            [&](const auto& child_data) {
                Mesh& child_mesh = *child_data.mesh;

                auto r = current_mesh.map_to_child(child_mesh, simplex);
#if !defined(NDEBUG)
                for (const auto& s : r) {
                    assert(child_mesh.is_valid_slow(s.tuple()));
                }
#endif

                return r;
            });


        // go over each child mesh / child simplices and run the node functor on them
        // then recurses this function onto the children
        for (size_t child_index = 0; child_index < child_datas.size(); ++child_index) {
            auto&& child_data = child_datas[child_index];
            auto&& simplices = mapped_child_simplices[child_index];
            Mesh& child_mesh_base = *child_data.mesh;

#if !defined(NDEBUG)
            for (const auto& s : simplices) {
                assert(child_mesh_base.is_valid_slow(s.tuple()));
            }
#endif

            auto child_mesh_variant =
                wmtk::utils::metaprogramming::as_mesh_variant(child_mesh_base);
            std::visit(
                [&](auto&& child_mesh_) noexcept {
                    auto&& child_mesh = child_mesh_.get();
                    using ChildType = std::decay_t<decltype(child_mesh)>;
#if defined(WMTK_MESH_VISITOR_ONLY_SUPPORTS_NONCONST_REFERENCE)
                    using ChildReturnType = GetReturnType_t<ChildType>;
#else
                    constexpr static bool ChildIsConst = std::is_const_v<ChildType>;
                    using ChildReturnType = GetReturnType_t<ChildIsConst, ChildType>;
#endif

                    constexpr static bool ChildHasReturn = !std::is_void_v<ChildReturnType>;
                    constexpr static long ChildDim =
                        wmtk::utils::metaprogramming::cell_dimension_v<ChildType>;

                        // std::visit compiles all combinations of meshes, and
                        // the behavior is undefined if MeshDim < ChildDim.
                        //
                        // we assert this to make sure the code is correct at
                        // runtime, we if constexpr after to make sure the
                        // compiler doesn't try to compile code in these cases
                    assert(MeshDim >= ChildDim);

                    if constexpr (MeshDim >= ChildDim) {
                        for (const simplex::Simplex& child_simplex : simplices) {
#if !defined(NDEBUG)
                            assert(child_mesh.is_valid_slow(child_simplex.tuple()));
#endif
                            run(child_mesh, child_simplex);

                            if constexpr (HasReturnCache && ChildHasReturn && CurHasReturn) {
                                if constexpr (HasEdgeFunctor) {
                                    auto parent_id = m_return_data.get_id(current_mesh, simplex);
                                    auto child_id = m_return_data.get_id(child_mesh, child_simplex);
                                    // spdlog::info(
                                    //     "MultiMeshVisitor[{}=>{}] adding to edges edge simplex {}
                                    //     " "child " "simplex{}",
                                    //     fmt::join(current_mesh.absolute_multi_mesh_id(), ","),
                                    //     fmt::join(child_mesh.absolute_multi_mesh_id(), ","),
                                    //     wmtk::utils::TupleInspector::as_string(
                                    //         std::get<1>(parent_id).tuple()),
                                    //     wmtk::utils::TupleInspector::as_string(
                                    //         std::get<1>(child_id).tuple()));
                                    edge_events.emplace_back(parent_id, child_id);
                                }
                            }
                        }
                    }
                },
                child_mesh_variant);
        }

        // after running on the chlidren, we finally run the operator and record the return data
        if constexpr (CurHasReturn) {
            auto current_return = visitor.m_node_functor(current_mesh, simplex);

            m_return_data.add(current_return, current_mesh, simplex);
        } else {
            visitor.m_node_functor(current_mesh, simplex);
        }
    }

    using KeyType = std::
        conditional_t<HasReturnCache, typename ReturnDataType::KeyType, std::tuple<const Mesh*>>;


    // cache of edge events that happened
    std::vector<std::tuple<KeyType, KeyType>> edge_events;
};


template <long cell_dimension, typename NodeFunctor, typename EdgeFunctor>
MultiMeshVisitor(std::integral_constant<long, cell_dimension>, NodeFunctor&&, EdgeFunctor&&)
    -> MultiMeshVisitor<cell_dimension, NodeFunctor, EdgeFunctor>;

template <long cell_dimension, typename NodeFunctor>
MultiMeshVisitor(std::integral_constant<long, cell_dimension>, NodeFunctor&&)
    -> MultiMeshVisitor<cell_dimension, NodeFunctor, std::monostate>;

template <typename NodeFunctor, typename EdgeFunctor>
MultiMeshVisitor(NodeFunctor&&, EdgeFunctor&&) -> MultiMeshVisitor<0, NodeFunctor, EdgeFunctor>;

template <typename NodeFunctor>
MultiMeshVisitor(NodeFunctor&&) -> MultiMeshVisitor<0, NodeFunctor, std::monostate>;

} // namespace wmtk::multimesh
