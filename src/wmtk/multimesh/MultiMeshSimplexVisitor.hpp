#pragma once
#include <type_traits>
#include <variant> //to get monostage
#include <wmtk/Mesh.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/operations/tri_mesh/EdgeOperationData.hpp>
#include <wmtk/simplex/NavigatableSimplex.hpp>
#include <wmtk/simplex/utils/MeshSimplexComparator.hpp>
#include <wmtk/utils/mesh_type_from_primitive_type.hpp>
#include <wmtk/utils/metaprogramming/MeshVariantTraits.hpp>
#include <wmtk/utils/metaprogramming/ReferenceWrappedFunctorReturnCache.hpp>
#include <wmtk/utils/metaprogramming/ReferenceWrappedFunctorReturnType.hpp>
#include <wmtk/utils/metaprogramming/as_mesh_variant.hpp>
#include <wmtk/utils/metaprogramming/cell_dimension.hpp>
#if !defined(NDEBUG)
#include <cassert>
#endif

// TODO: extend this visitor to support const and non-const references
#define WMTK_MESH_VISITOR_ONLY_SUPPORTS_NONCONST_REFERENCE

namespace wmtk::multimesh {

// if NodeFunctor returns a value then
template <typename MMVisitor>
class MultiMeshSimplexVisitorExecutor;

template <int64_t cell_dimension_, typename NodeFunctor_>
class MultiMeshSimplexVisitor
{
public:
    using MeshVariantTraits = wmtk::utils::metaprogramming::MeshVariantTraits;
    using NodeFunctor = NodeFunctor_;
    constexpr static int64_t cell_dimension = cell_dimension_;

    constexpr static bool HasReturnCache =
        !wmtk::utils::metaprogramming::
            all_return_void_v<NodeFunctor, MeshVariantTraits, simplex::Simplex>;
    using ReturnDataType =
        wmtk::utils::metaprogramming::ReferenceWrappedFunctorReturnCacheCustomComparator<
            NodeFunctor,
            MeshVariantTraits,
            wmtk::simplex::utils::MeshSimplexComparator,
            simplex::NavigatableSimplex>;
    using CacheType = ReturnDataType;

    using TypeHelper = wmtk::utils::metaprogramming::detail::ReferenceWrappedFunctorReturnType<
        NodeFunctor,
        MeshVariantTraits::AllReferenceTuple,
        wmtk::simplex::Simplex>;
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


    /* @brief constructor that takes in the node and edge functors
     *
     * @param f The functor that will be run on each mesh in the tree
     * */
    MultiMeshSimplexVisitor(NodeFunctor&& f)
        : m_node_functor(f)
    {}

    /* @brief utility constructor that delegates a constant for class template arugment deduction
     * @param _ deduction hint that helps pick cell_dimension
     * @param f The functor that will be run on each mesh in the tree
     * */
    MultiMeshSimplexVisitor(std::integral_constant<int64_t, cell_dimension>, NodeFunctor&& f)
        : MultiMeshSimplexVisitor(std::forward<NodeFunctor>(f))
    {}


    template <typename MMVisitor_>
    friend class MultiMeshSimplexVisitorExecutor;
    using Executor =
        MultiMeshSimplexVisitorExecutor<MultiMeshSimplexVisitor<cell_dimension, NodeFunctor>>;

    /* @brief executes the node functor (and potentially edge functor) from the subtree of the input
     * node
     * @param mesh the mesh in the tree that the operation needs to run from
     * @param simplex the simplex on the input mesh that we want to run the operation from
     * @return a ReferenceWrappedFunctorReturnCache that lets one request (mesh,simplex) ->
     * NodeFunctor Return type
     * */
    template <typename MeshType>
    void execute_mesh(MeshType&& mesh, const simplex::NavigatableSimplex& simplex)
    {
        static_assert(
            !std::is_same_v<std::decay_t<MeshType>, Mesh>,
            "Don't pass in a mesh, use variant/visitor to get its derived type");
        Executor exec(*this);
        exec.execute(std::forward<MeshType>(mesh), simplex);
        if constexpr (HasReturnCache) {
            m_cache = std::move(exec.m_return_data);
            m_edge_events = std::move(exec.edge_events);
        }
    }

    /* @brief executes the node functor (and potentially edge functor) from the entire graph
     * @param mesh some mesh in the tree that the operation needs to run from
     * @param simplex the simplex on the input mesh that we want to run the operation from
     * @return a ReferenceWrappedFunctorReturnCache that lets one request (mesh,simplex) ->
     * NodeFunctor Return type
     * */
    // even if you try to use an interior mesh node this always just uses the root
    void execute_from_root(Mesh& mesh, const simplex::NavigatableSimplex& simplex)
    {
        // if the user passed in a mesh class lets try re-invoking with a derived type
        Mesh& root_base_mesh = mesh.get_multi_mesh_root();
        auto mesh_root_variant = wmtk::utils::metaprogramming::as_mesh_variant(root_base_mesh);

        const simplex::Simplex root_simplex_nonav = mesh.map_to_root(simplex);
        assert(root_base_mesh.is_valid(root_simplex_nonav.tuple()));
        const simplex::NavigatableSimplex root_simplex(root_base_mesh, root_simplex_nonav);
        assert(root_base_mesh.is_valid(root_simplex.tuple()));
        Executor exec(*this);
        std::visit([&](auto&& root) { execute_mesh(root.get(), root_simplex); }, mesh_root_variant);
    }

    const CacheType& cache() const { return m_cache; }
    CacheType take_cache() { return std::move(m_cache); }
    auto node_events() const { return m_cache.keys(); }
    const auto& edge_events() const { return m_edge_events; }

protected:
    NodeFunctor m_node_functor;
    CacheType m_cache;

    using KeyType = std::
        conditional_t<HasReturnCache, typename ReturnDataType::KeyType, std::tuple<const Mesh*>>;


    // cache of edge events that happened
    std::vector<std::tuple<KeyType, KeyType>> m_edge_events;
};

// if NodeFunctor returns a value then
template <typename MMVisitor>
class MultiMeshSimplexVisitorExecutor
{
public:
    constexpr static int64_t cell_dimension = MMVisitor::cell_dimension;
    using MeshVariantTraits = wmtk::utils::metaprogramming::MeshVariantTraits;
    using NodeFunctor = typename MMVisitor::NodeFunctor;
    template <typename T>
    using GetReturnType_t = typename MMVisitor::template GetReturnType_t<T>;
    // template <bool IsConst, typename MeshType>

    using ReturnDataType =
        wmtk::utils::metaprogramming::ReferenceWrappedFunctorReturnCacheCustomComparator<
            NodeFunctor,
            MeshVariantTraits,
            wmtk::simplex::utils::MeshSimplexComparator,
            simplex::NavigatableSimplex>;
    constexpr static bool HasReturnCache =
        !wmtk::utils::metaprogramming::
            all_return_void_v<NodeFunctor, MeshVariantTraits, simplex::Simplex>;

    MultiMeshSimplexVisitorExecutor(const MMVisitor& v)
        : visitor(v)
    {}

    ReturnDataType m_return_data;
    const MMVisitor& visitor;
    using KeyType = std::
        conditional_t<HasReturnCache, typename ReturnDataType::KeyType, std::tuple<const Mesh*>>;


    // cache of edge events that happened
    std::vector<std::tuple<KeyType, KeyType>> edge_events;


    /* @brief runs the node functor on every node in the subgraph and then runs hte edge functor if
     * it exists
     * @param mesh the mesh whose subgraph will be run
     * @param simplex the simplex whose subgraph will be run
     * */
    template <typename MeshType>
    void execute(MeshType&& mesh, const simplex::NavigatableSimplex& simplex)
    {
        static_assert(std::is_base_of_v<Mesh, std::decay_t<MeshType>>);
        run(std::forward<MeshType>(mesh), simplex);
    }


private:
    /* @brief runs the node functor on every node in the subgraph
     * @param mesh the mesh whose subgraph will be run
     * @param simplex the simplex whose subgraph will be run
     * */
    template <typename MeshType_>
    void run(MeshType_&& current_mesh, const simplex::NavigatableSimplex& simplex)
    {
        assert(current_mesh.is_valid(simplex.tuple()));
        using MeshType = std::decay_t<MeshType_>;


        // short circuit operations that happen below the desired dimension
        constexpr static int64_t MeshDim = wmtk::utils::metaprogramming::cell_dimension_v<MeshType>;
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
        std::vector<std::vector<simplex::NavigatableSimplex>> mapped_child_simplices;
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

                const std::vector<simplex::Simplex> _r =
                    current_mesh.map_to_child(child_mesh, simplex);
                std::vector<simplex::NavigatableSimplex> r;
                std::transform(
                    _r.begin(),
                    _r.end(),
                    std::back_inserter(r),
                    [&](const simplex::Simplex& s) {
                        return simplex::NavigatableSimplex(child_mesh, s);
                    });
#if !defined(NDEBUG)
                for (const auto& s : r) {
                    assert(child_mesh.is_valid(s.tuple()));
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
                assert(child_mesh_base.is_valid(s.tuple()));
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
                    constexpr static int64_t ChildDim =
                        wmtk::utils::metaprogramming::cell_dimension_v<ChildType>;

                    // std::visit compiles all combinations of meshes, and
                    // the behavior is undefined if MeshDim < ChildDim.
                    //
                    // we assert this to make sure the code is correct at
                    // runtime, we if constexpr after to make sure the
                    // compiler doesn't try to compile code in these cases
                    assert(MeshDim >= ChildDim);

                    if constexpr (MeshDim >= ChildDim) {
                        for (const simplex::NavigatableSimplex& child_simplex : simplices) {
                            assert(child_mesh.is_valid(child_simplex.tuple()));

                            run(child_mesh, child_simplex);

                            if constexpr (HasReturnCache && ChildHasReturn && CurHasReturn) {
                                auto parent_id = m_return_data.get_id(current_mesh, simplex);
                                auto child_id = m_return_data.get_id(child_mesh, child_simplex);
                                edge_events.emplace_back(parent_id, child_id);
                            }
                        }
                    }
                },
                child_mesh_variant);
        }

        // after running on the chlidren, we finally run the operator and record the return data
        if constexpr (CurHasReturn) {
            auto current_return = visitor.m_node_functor(current_mesh, simplex);

            m_return_data.add(std::move(current_return), current_mesh, simplex);
        } else {
            visitor.m_node_functor(current_mesh, simplex);
        }
    }
};


template <int64_t cell_dimension, typename NodeFunctor>
MultiMeshSimplexVisitor(std::integral_constant<int64_t, cell_dimension>, NodeFunctor&&)
    -> MultiMeshSimplexVisitor<cell_dimension, NodeFunctor>;

template <typename NodeFunctor>
MultiMeshSimplexVisitor(NodeFunctor&&) -> MultiMeshSimplexVisitor<0, NodeFunctor>;


} // namespace wmtk::multimesh
