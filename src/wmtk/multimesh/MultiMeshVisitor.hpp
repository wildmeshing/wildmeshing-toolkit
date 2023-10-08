#pragma once
#include <spdlog/spdlog.h>
#include <type_traits>
#include <variant> //to get monostage
#include <wmtk/Mesh.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/as_mesh_variant.hpp>
#include <wmtk/utils/mesh_type_from_primitive_type.hpp>


namespace wmtk::multimesh {

// if NodeFunctor returns a value then
template <typename MMVisitor, typename ReturnMeshType>
class MultiMeshVisitorExecutor;

template <typename NodeFunctor, typename EdgeFunctor = std::monostate>
class MultiMeshVisitor
{
    template <typename MeshType>
    using GetReturnType_t = std::invoke_result_t<NodeFunctor, MeshType&, const simplex::Simplex&>;
    constexpr static bool HasEdgeFunctor = !std::is_same_v<EdgeFunctor, std::monostate>;
    MultiMeshVisitor(NodeFunctor&& f, EdgeFunctor&& ef)
        : m_node_functor(f)
        , m_edge_functor(ef)
    {}
    MultiMeshVisitor(NodeFunctor&& f)
        : m_node_functor(f)
    {}

    // even if you try to use an interior mesh node this always just uses the root
    template <typename MeshType>
    auto execute_from_root(MeshType& mesh, const simplex::Simplex& simplex) const
        -> GetReturnType_t<MeshType>
    {
        // if the user passed in a mesh class lets try re-invoking with a derived type
        if constexpr (std::is_same_v<std::decay_t<MeshType>, Mesh>) {
            auto mesh_variant = wmtk::utils::as_mesh_variant(mesh);
            std::visit(
                [&](const auto& mesh) {
                    // assert(!std::is_same_v<std::decay_t<MeshType>, Mesh>);
                    execute_from_root(mesh, simplex);
                },
                mesh_variant);
        } else {
            MultiMeshVisitorExecutor exec(*this, mesh, simplex);
            if (!mesh.is_multi_mesh_root()) {
                Mesh& root = mesh.get_multi_mesh_root();
                auto mesh_root_variant = wmtk::utils::as_mesh_variant(root);
                const simplex::Simplex root_simplex = mesh.map_to_root(simplex);
                return std::visit(
                    [&](auto& root) { return exec.execute(root, root_simplex); },
                    mesh_root_variant);
            } else {
                return exec.execute(mesh, simplex);
            }
        }
    }
    // execute on teh subtree (if you want the entire tree use execute_from_root)
    //
    template <typename MeshType>
    auto execute_mesh(MeshType& mesh, const simplex::Simplex& simplex) const
    {
        // if the user passed in a mesh class lets try re-invoking with a derived type
        if constexpr (std::is_same_v<std::decay_t<MeshType>, Mesh>) {
            auto mesh_variant = wmtk::utils::as_mesh_variant(mesh);
            std::visit(
                [&](const auto& mesh) {
                    // assert(!std::is_same_v<std::decay_t<MeshType>, Mesh>);
                    execute_from_root(mesh, simplex);
                },
                mesh_variant);
        } else {
            MultiMeshVisitorExecutor exec(this, mesh, simplex);
            return exec.execute(mesh, simplex);
        }
    }
    NodeFunctor m_node_functor;
    EdgeFunctor m_edge_functor;
};

// if NodeFunctor returns a value then
template <typename MMVisitor, typename ReturnMeshType>
class MultiMeshVisitorExecutor
{
public:
    using NodeFunctor = typename MMVisitor::NodeFunctior;
    using EdgeFunctor = typename MMVisitor::EdgeFunctor;
    template <typename MeshType>
    using GetReturnType_t = std::invoke_result_t<NodeFunctor, MeshType&, const simplex::Simplex&>;
    template <typename MeshType>
    constexpr static bool HasReturnValue_v = !std::is_void_v<GetReturnType_t<MeshType>>;

    MultiMeshVisitor& visitor;

    const MeshType& return_mesh;
    const Simplex& return_simplex;
    constexpr static bool HasEdgeFunctor = MMVisitor::HasEdgeFunctor;

    constexpr static bool FinalHasReturn = HasReturnValue_v<ReturnMeshType>;
    using FinalReturnType = GetReturnType_t<ReturnMeshType>;

    // there's some code where i want to store an uninitialized member so easier to use monospace
    // instead of void
    using FinalReturnOpt = std::conditional_t<
        std::is_void_v<FinalReturnType>,
        std::monostate,
        std::optional<FinalReturnType>>;


    // checks whether return info data is the same as some mesh/simplex pair
    template <typename MeshType>
    bool is_same_as_final(const MeshType& mesh, const Simplex& s) const
    {
        return std::is_same_v<MeshType, ReturnMeshType> &&
               static_cast<const Mesh*>(&return_mesh) == static_cast<const Mesh*>(&mesh) &&
               return_simplex == s;
    }


    template <typename MeshType>
    auto execute(MeshType& mesh, const simplex::Simplex& simplex) const -> FinalReturnType
    {
        using CurReturnType = GetReturnType_t<MeshType>;

        constexpr static bool CurHasReturn = !std::is_void_v<CurReturnType>;
        if constexpr (FinalHasReturn) {
            if constexpr (CurHasReturn) {
                // return value is std::tuple<CurReturn, std::optional<BaseReturn>>
                auto&& [cur, base_opt] = run(mesh, simplex);

                assert(bool(base_opt));
                return base_opt.get_value();

            } else {
                // return value is std::optional<BaseReturn>
                auto base_opt = run(mesh, simplex);
                assert(bool(base_opt));
                return base_opt.get_value();
            }
        } else {
            run(mesh, simplex);
        }
    }


private:
    // template <typename MeshType, typename InputInfo>
    // auto execute_mesh(MeshType& mesh, const simplex::Simplex& simplex, const InputInfo&
    // return_mesh)
    //     const -> TupleGetReturnType_t<InputInfo>
    //{
    //     using ReturnType = GetReturnType_t<ReturnMesh>;
    //     constexpr static bool NoReturn = !HasReturnValue_v<ReturnMesh>;

    //    if constexpr (NoReturn) {
    //        execute_T(mesh, simplex, return_mesh);
    //    } else {
    //        return execute_T(mesh, simplex, return_mesh);
    //    }
    //}

    template <typename MeshType>
    auto run(MeshType& current_mesh, const simplex::Simplex& simplex) const
    {
        using CurReturnType = GetReturnType_t<MeshType>;

        constexpr static bool CurHasReturn = !std::is_void_v<CurReturnType>;

        // this is monostate if we return nothing, an optional if we do want to return something
        FinalReturnOpt return_data;


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


        if constexpr (FinalHasReturn) {
            auto current_return = m_node_functor(current_mesh, simplex);

            // try taking the return data if it's possible
            constexpr static bool SameTypeAsReturn = std::is_same_v<MeshType, ReturnMeshType>;

            bool cur_is_base = false;

            if constexpr (SameTypeAsReturn) {
                if (is_same_as_final(current_mesh, simplex)) {
                    cur_is_base = true;
                    return_data = current_return;
                }
            }


            for (size_t child_index = 0; child_index < child_datas.size(); ++child_index) {
                auto& child_data = child_datas[child_index];
                auto& simplices = mapped_child_simplices[child_index];
                Mesh& child_mesh_base = *child_data.mesh;
                auto chld_mesh_variant = wmtk::utils::as_mesh_variant(child_mesh_base);
                std::visit(
                    [&](const auto& child_mesh) {
                        using ChildMeshType = std::unwrap_ref_decay_t<decltype(child_mesh)>;
                        using ChildReturnType = GetReturnType_t<ChildMeshType>;


                        for (const simplex::Simplex& child_simplex : simplices) {
                            if constexpr (FinalHasReturn) {
                                auto return_opt = run_over_edge(
                                    current_mesh,
                                    current_return,
                                    child_mesh,
                                    child_simplex);
                                // if the current node is the desired mesh then we short circuit
                                // this check otherwise we check return_opt - which might have
                                // the desired value
                                if (!cur_is_base && return_opt) {
                                    return_data = return_opt;
                                }
                            } else {
                                run_over_edge(
                                    current_mesh,
                                    current_return,
                                    child_mesh,
                                    child_simplex);
                            }
                        }
                    },
                    child_mesh_variant);
            }


            if constexpr (FinalHasReturn) {
                // if we have return data + this op returns something send back both as a tuple
                return std::make_tuple(current_return, return_data);
            } else {
                // if we didn't need to send back return data then just send the value
                return current_return;
            }
        } else {
            m_node_functor(current_mesh, simplex);
            for (size_t child_index = 0; child_index < child_datas.size(); ++child_index) {
                auto& child_data = child_datas[child_index];
                auto& simplices = mapped_child_simplices[child_index];
                Mesh& child_mesh_base = *child_data.mesh;
                auto chld_mesh_variant = wmtk::utils::as_mesh_variant(child_mesh_base);
                std::visit(
                    [&](const auto& child_mesh) {
                        using ChildMeshType = std::unwrap_ref_decay_t<decltype(child_mesh)>;
                        using ChildReturnType = GetReturnType_t<ChildMeshType>;

                        for (const simplex::Simplex& child_simplex : simplices) {
                            if constexpr (FinalHasReturn) {
                                auto return_opt = run(child_mesh, child_simplex);
                                // if the current node is the desired mesh then we short circuit
                                // this check otherwise we check return_opt - which might have
                                // the desired value
                                if (return_opt) {
                                    return_data = return_opt;
                                }
                            } else {
                                run(child_mesh, child_simplex);
                            }
                        }
                    },
                    child_mesh_variant);
            }
            if constexpr (FinalHasReturn) {
                return return_data;
            } else {
                return;
            }
        }
    }


    template <typename ParentType, typename ParentReturn, typename ChildMesh>
    auto run_over_edge(
        ParentType& parent_mesh,
        const ParentReturn& parent_return,
        ChildMesh& child_mesh,
        const simplex::Simplex& child_simplex) const
    {
        using ChildReturn = GetReturnType_t<ChildMesh>;

        constexpr static bool ChildHasReturn = !std::is_void_v<ChildReturnType>;

        if constexpr (ChildHasReturn) {
            auto child_return = run(child_mesh, child_simplex);
            spdlog::info("Has return data! should do edge functor {}", HasEdgeFunctor);

            if constexpr (HasEdgeFunctor) {
                m_edge_functor(parent, parent_return, child_mesh, child_return);
            }
            if constexpr (FinalHasReturn) {
                // its a tuple of child return and final return
                return std::get<1>(child_return);
            }
        } else {
            return run(child_mesh, child_simplex);
        }
    }
};


template <typename NodeFunctor, typename EdgeFunctor>
MultiMeshVisitor(NodeFunctor&&, EdgeFunctor&&) -> MultiMeshVisitor<NodeFunctor, EdgeFunctor>;

template <typename NodeFunctor>
MultiMeshVisitor(NodeFunctor&&) -> MultiMeshVisitor<NodeFunctor, std::monostate>;

} // namespace wmtk::multimesh
