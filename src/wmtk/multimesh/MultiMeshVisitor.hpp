#pragma once
#include <spdlog/spdlog.h>
#include <type_traits>
#include <variant> //to get monostage
#include <wmtk/Mesh.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/mesh_type_from_primitive_type.hpp>
#include <wmtk/utils/metaprogramming/as_mesh_variant.hpp>
#include "utils/CachedMeshVariantReturnValues.hpp"


namespace wmtk::multimesh {

// if NodeFunctor returns a value then
template <typename MMVisitor>
class MultiMeshVisitorExecutor;

template <typename NodeFunctor_, typename EdgeFunctor_ = std::monostate>
class MultiMeshVisitor
{
public:
    using NodeFunctor = NodeFunctor_;
    using EdgeFunctor = EdgeFunctor_;
    constexpr static bool HasEdgeFunctor = !std::is_same_v<EdgeFunctor, std::monostate>;

    using ReturnType = utils::CachedMeshVariantReturnValues<NodeFunctor, Simplex>;

    MultiMeshVisitor(NodeFunctor&& f, EdgeFunctor&& ef)
        : m_node_functor(f)
        , m_edge_functor(ef)
    {}
    MultiMeshVisitor(NodeFunctor&& f)
        : m_node_functor(f)
    {}


    template <typename MMVisitor_>
    friend class MultiMeshVisitorExecutor;
    using Executor = MultiMeshVisitorExecutor<MultiMeshVisitor<NodeFunctor, EdgeFunctor>>;

    // even if you try to use an interior mesh node this always just uses the root
    template <typename MeshType>
    ReturnType execute_from_root(MeshType& mesh, const simplex::Simplex& simplex) const
    {
        static_assert(
            !std::is_same_v<std::decay_t<MeshType>, Mesh>,
            "Don't pass in a mesh, use variant/visitor to get its derived type");
        // if the user passed in a mesh class lets try re-invoking with a derived type
        Mesh& root = mesh.get_multi_mesh_root();
        auto mesh_root_variant = wmtk::utils::metaprogramming::as_mesh_variant(root);
        const simplex::Simplex root_simplex = mesh.map_to_root(simplex);
        return std::visit(
            [&](auto& root) { return execute_mesh(root, root_simplex); },
            mesh_root_variant);
    }
    // execute on teh subtree (if you want the entire tree use execute_from_root)
    //
    template <typename MeshType>
    ReturnType execute_mesh(MeshType& mesh, const simplex::Simplex& simplex) const
    {
        static_assert(
            !std::is_same_v<std::decay_t<MeshType>, Mesh>,
            "Don't pass in a mesh, use variant/visitor to get its derived type");
        Executor exec(this);
        exec.execute(mesh, simplex);
        return exec.m_return_data;
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
    using NodeFunctor = typename MMVisitor::NodeFunctor;
    using EdgeFunctor = typename MMVisitor::EdgeFunctor;
    using TypeHelper = utils::
        ReturnVariantHelper<NodeFunctor, wmtk::utils::metaprogramming::MeshVariantType, Simplex>;
    template <bool IsConst, typename MeshType>
    using GetReturnType_t = typename TypeHelper::ReturnType_const<IsConst, MeshType>;
    template <bool IsConst, typename MeshType>
    constexpr static bool HasReturnValue_v = !std::is_void_v<GetReturnType_t<IsConst, MeshType>>;

    using ReturnDataType = utils::CachedMeshVariantReturnValues<NodeFunctor, Simplex>;

    MultiMeshVisitorExecutor(const MMVisitor& v)
        : visitor(v)
    {}

    const MMVisitor& visitor;


    ReturnDataType m_return_data;

    constexpr static bool HasEdgeFunctor = MMVisitor::HasEdgeFunctor;


    template <typename MeshType>
    void execute(MeshType& mesh, const simplex::Simplex& simplex) const
    {
        run(mesh, simplex);
    }


private:
    template <typename MeshType_>
    auto run(MeshType_&& current_mesh, const simplex::Simplex& simplex) const
    {
        using MeshType = std::decay_t<MeshType_>;
        constexpr static bool CurIsConst = std::is_const_v<MeshType>;
        using CurReturnType = GetReturnType_t<CurIsConst, MeshType>;

        constexpr static bool CurHasReturn = !std::is_void_v<CurReturnType>;


        // pre-compute all of  the child tuples in case the node functor changes the mesh that
        // breaks the traversal down
        auto& child_datas = current_mesh.m_multi_mesh_manager.children();
        std::vector<std::vector<Simplex>> mapped_child_simplices;
        mapped_child_simplices.reserve(child_datas.size());
        std::transform(
            child_datas.begin(),
            child_datas.end(),
            std::back_inserter(mapped_child_simplices),
            [&](const auto& child_data) {
                Mesh& child_mesh = *child_data.mesh;

                return current_mesh.map_to_child(child_mesh, simplex);
            });


        if constexpr (CurHasReturn) {
            auto current_return = m_node_functor(current_mesh, simplex);

            m_return_data.add(current_mesh, current_return);


            for (size_t child_index = 0; child_index < child_datas.size(); ++child_index) {
                auto& child_data = child_datas[child_index];
                auto& simplices = mapped_child_simplices[child_index];
                Mesh& child_mesh_base = *child_data.mesh;
                auto child_mesh_variant =
                    wmtk::utils::metaprogramming::as_mesh_variant(child_mesh_base);
                std::visit(
                    [&](const auto& child_mesh) {
                        using ChildMeshType_ = decltype(child_mesh);
                        constexpr static bool ChildIsConst = std::is_const_v<ChildMeshType_>;
                        using ChildMeshType =
                            wmtk::utils::metaprogramming::unwrap_ref_decay_t<ChildMeshType_>;
                        using ChildReturnType = GetReturnType_t<ChildIsConst, ChildMeshType>;


                        // for (const simplex::Simplex& child_simplex : simplices) {
                        //     run_over_edge(current_mesh, current_return, child_mesh,
                        //     child_simplex);
                        // }
                    },
                    child_mesh_variant);
            }
            return current_return;
        } else {
            m_node_functor(current_mesh, simplex);
            for (size_t child_index = 0; child_index < child_datas.size(); ++child_index) {
                auto& child_data = child_datas[child_index];
                auto& simplices = mapped_child_simplices[child_index];
                Mesh& child_mesh_base = *child_data.mesh;
                auto child_mesh_variant =
                    wmtk::utils::metaprogramming::as_mesh_variant(child_mesh_base);
                std::visit(
                    [&](const auto& child_mesh) {
                        for (const simplex::Simplex& child_simplex : simplices) {
                            run(child_mesh, child_simplex);
                        }
                    },
                    child_mesh_variant);
            }
        }
    }


    template <typename ParentType, typename ParentReturn, typename ChildMesh>
    void run_over_edge(
        ParentType& parent_mesh,
        const ParentReturn& parent_return,
        ChildMesh& child_mesh,
        const simplex::Simplex& child_simplex) const
    {
        using ChildReturnType = GetReturnType_t<false, ChildMesh>;

        constexpr static bool ChildHasReturn = !std::is_void_v<ChildReturnType>;

        auto child_return = run(child_mesh, child_simplex);
        if constexpr (ChildHasReturn && HasEdgeFunctor) {
            m_edge_functor(parent_mesh, parent_return, child_mesh, child_return);
        }
    }
};


template <typename NodeFunctor, typename EdgeFunctor>
MultiMeshVisitor(NodeFunctor&&, EdgeFunctor&&) -> MultiMeshVisitor<NodeFunctor, EdgeFunctor>;

template <typename NodeFunctor>
MultiMeshVisitor(NodeFunctor&&) -> MultiMeshVisitor<NodeFunctor, std::monostate>;

} // namespace wmtk::multimesh
