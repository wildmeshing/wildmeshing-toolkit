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
#if !defined(NDEBUG)
#include <cassert>
#endif

// TODO: extend this visitor to support const and non-const references
#define WMTK_MESH_VISITOR_ONLY_SUPPORTS_NONCONST_REFERENCE

namespace wmtk::multimesh {

// if NodeFunctor returns a value then
template <typename MMVisitor>
class MultiMeshVisitorExecutor;

template <typename NodeFunctor_>
class MultiMeshVisitor
{
public:
    using MeshVariantTraits = wmtk::utils::metaprogramming::MeshVariantTraits;
    using NodeFunctor = NodeFunctor_;

    using ReturnDataType = wmtk::utils::metaprogramming::
        ReferenceWrappedFunctorReturnCache<NodeFunctor, MeshVariantTraits>;
    using CacheType = wmtk::utils::metaprogramming::
        ReferenceWrappedFunctorReturnCache<NodeFunctor, MeshVariantTraits>;

    using TypeHelper = wmtk::utils::metaprogramming::detail::
        ReferenceWrappedFunctorReturnType<NodeFunctor, MeshVariantTraits::AllReferenceTuple>;


    /* @brief constructor that takes in the node and edge functors
     *
     * @param f The functor that will be run on each mesh in the tree
     * */
    MultiMeshVisitor(NodeFunctor&& f)
        : m_node_functor(f)
    {}


    template <typename MMVisitor_>
    friend class MultiMeshVisitorExecutor;
    using Executor = MultiMeshVisitorExecutor<MultiMeshVisitor<NodeFunctor>>;

    /* @brief executes the node functor (and potentially edge functor) from the subtree of the input
     * node
     * @param mesh the mesh in the tree that the operation needs to run from
     * @param simplex the simplex on the input mesh that we want to run the operation from
     * @return a ReferenceWrappedFunctorReturnCache that lets one request (mesh,simplex) ->
     * NodeFunctor Return type
     * */
    template <typename MeshType>
    void execute_mesh(MeshType&& mesh)
    {
        static_assert(
            !std::is_same_v<std::decay_t<MeshType>, Mesh>,
            "Don't pass in a mesh, use variant/visitor to get its derived type");
        Executor exec(*this);
        exec.execute(std::forward<MeshType>(mesh));
    }

    /* @brief executes the node functor (and potentially edge functor) from the entire graph
     * @param mesh some mesh in the tree that the operation needs to run from
     * @param simplex the simplex on the input mesh that we want to run the operation from
     * @return a ReferenceWrappedFunctorReturnCache that lets one request (mesh,simplex) ->
     * NodeFunctor Return type
     * */
    // even if you try to use an interior mesh node this always just uses the root
    void execute_from_root(Mesh& mesh)
    {
        // if the user passed in a mesh class lets try re-invoking with a derived type
        Mesh& root_base_mesh = mesh.get_multi_mesh_root();
        auto mesh_root_variant = wmtk::utils::metaprogramming::as_mesh_variant(root_base_mesh);
        Executor exec(*this);
        std::visit([&](auto&& root) { execute_mesh(root.get()); }, mesh_root_variant);
    }


protected:
    NodeFunctor m_node_functor;
};

// if NodeFunctor returns a value then
template <typename MMVisitor>
class MultiMeshVisitorExecutor
{
public:
    using MeshVariantTraits = wmtk::utils::metaprogramming::MeshVariantTraits;
    using NodeFunctor = typename MMVisitor::NodeFunctor;
    // template <bool IsConst, typename MeshType>

    MultiMeshVisitorExecutor(const MMVisitor& v)
        : visitor(v)
    {}

    const MMVisitor& visitor;


    /* @brief runs the node functor on every node in the subgraph and then runs hte edge functor if
     * it exists
     * @param mesh the mesh whose subgraph will be run
     * @param simplex the simplex whose subgraph will be run
     * */
    template <typename MeshType>
    void execute(MeshType&& mesh)
    {
        static_assert(std::is_base_of_v<Mesh, std::decay_t<MeshType>>);
        run(std::forward<MeshType>(mesh));
    }


private:
    /* @brief runs the node functor on every node in the subgraph
     * @param mesh the mesh whose subgraph will be run
     * @param simplex the simplex whose subgraph will be run
     * */
    template <typename MeshType_>
    void run(MeshType_&& current_mesh)
    {
        using MeshType = std::decay_t<MeshType_>;


        // short circuit operations that happen below the desired dimension
        constexpr static int64_t MeshDim = wmtk::utils::metaprogramming::cell_dimension_v<MeshType>;


        // pre-compute all of  the child tuples in case the node functor changes the mesh that
        // breaks the traversal down
        auto& child_datas = current_mesh.m_multi_mesh_manager.children();

        // go over each child mesh / child simplices and run the node functor on them
        // then recurses this function onto the children
        for (size_t child_index = 0; child_index < child_datas.size(); ++child_index) {
            auto&& child_data = child_datas[child_index];
            Mesh& child_mesh_base = *child_data.mesh;


            auto child_mesh_variant =
                wmtk::utils::metaprogramming::as_mesh_variant(child_mesh_base);
            std::visit(
                [&](auto&& child_mesh_) noexcept {
                    auto&& child_mesh = child_mesh_.get();
                    using ChildType = std::decay_t<decltype(child_mesh)>;

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
                        run(child_mesh);
                    }
                },
                child_mesh_variant);
        }

        visitor.m_node_functor(current_mesh);
    }
};


template <typename NodeFunctor>
MultiMeshVisitor(NodeFunctor&&) -> MultiMeshVisitor<NodeFunctor>;


} // namespace wmtk::multimesh
