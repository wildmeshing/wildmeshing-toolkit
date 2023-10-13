#pragma once
#include <spdlog/spdlog.h>
#include <type_traits>
#include <variant> //to get monostage
#include <wmtk/Mesh.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/metaprogramming/as_mesh_variant.hpp>
#include <wmtk/utils/mesh_type_from_primitive_type.hpp>


namespace wmtk::multimesh {

template <typename NodeFunctor>
class MultiMeshVisitorReturnData;
// if NodeFunctor returns a value then
template <typename MMVisitor, typename ReturnMeshType>
class MultiMeshVisitorExecutor;

template <typename NodeFunctor, typename EdgeFunctor = std::monostate>
class MultiMeshVisitor
{
    constexpr static bool HasEdgeFunctor = !std::is_same_v<EdgeFunctor, std::monostate>;

    using MultiMeshVisitorReturnDataType = MultiMeshVisitorReturnData<NodeFunctor>;

    MultiMeshVisitor(NodeFunctor&& f, EdgeFunctor&& ef)
        : m_node_functor(f)
        , m_edge_functor(ef)
    {}
    MultiMeshVisitor(NodeFunctor&& f)
        : m_node_functor(f)
    {}

    // even if you try to use an interior mesh node this always just uses the root
    template <typename MeshType>
    MultiMeshVisitorReturnDataType execute_from_root(
        MeshType& mesh,
        const simplex::Simplex& simplex) const->GetReturnType_t<MeshType>
    {
        static_assert(
            !std::is_same_v<std::decay_t<MeshType>, Mesh>,
            "Don't pass in a mesh, use variant/visitor to get its derived type");
        // if the user passed in a mesh class lets try re-invoking with a derived type
        MultiMeshVisitorExecutor exec(*this, mesh, simplex);
        Mesh& root = mesh.get_multi_mesh_root();
        auto mesh_root_variant = wmtk::utils::metaprogramming::as_mesh_variant(root);
        const simplex::Simplex root_simplex = mesh.map_to_root(simplex);
        std::visit([&](auto& root) { exec.execute(root, root_simplex); }, mesh_root_variant);

        return exec.m_return_data;
    }
    // execute on teh subtree (if you want the entire tree use execute_from_root)
    //
    template <typename MeshType>
    MultiMeshVisitorReturnDataType execute_mesh(MeshType& mesh, const simplex::Simplex& simplex)
        const
    {
        static_assert(
            !std::is_same_v<std::decay_t<MeshType>, Mesh>,
            "Don't pass in a mesh, use variant/visitor to get its derived type");
        MultiMeshVisitorExecutor exec(this, mesh, simplex);
        exec.execute(mesh, simplex);
        return exec.m_return_data;
    }


    NodeFunctor m_node_functor;
    EdgeFunctor m_edge_functor;
};


template <typename NodeFunctor>
class MultiMeshVisitorReturnData
{
public:
    template <typename MeshType>
    using GetReturnType_t = std::decay_t<std::invoke_result_t<
        NodeFunctor,
        std::unwrap_ref_decay_t<MeshType>&,
        const simplex::Simplex&>>;

    using MeshVariantType = utils::MeshVariantType;


    template <typename T>
    using ReturnTupleType = void;
    template <typename MeshTypes...>
    using ReturnTupleType<std::variant<MeshTypes, ...>> =
        std::variant<GetReturnType_t<MeshTypes>, ...>;

    template <typename MeshType, ReturnType>
    void add(const MeshType& mesh, ReturnType&& return_data)
    {
        using ReturnType_t = std::decay_t<ReturnType>;
        static_assert(
            !std::is_same_v<std::decay_t<MeshType>, Mesh>,
            "Don't pass in a mesh, use variant/visitor to get its derived type");
        // if the user passed in a mesh class lets try re-invoking with a derived type
        auto id = mesh.absolute_multi_mesh_id();
        using ExpectedReturnType = GetReturnType_t<MeshType>;

        static_assert(
            std::is_convertible<ReturnData_t, ExpectedReturnType>,
            "Second argument should be the return value of a NodeFunctor (or convertible at "
            "least) ");


        m_data[id] = ReturnTypeVariant(std::in_place_type_t<ExpectedReturnType>, return_data);
    }

    const auto& get_variant(const Mesh& mesh) const
    {
        auto id = mesh.absolute_multi_mesh_id();
        return m_data.at(id);
    }

    template <typename MeshType>
    auto get(const MeshType& mesh) const
    {
        static_assert(
            !std::is_same_v<std::decay_t<MeshType>, Mesh>,
            "Don't pass in a mesh, use variant/visitor to get its derived type");
        using ExpectedReturnType = GetReturnType_t<MeshType>;

        return std::get<ExpectedReturnType>(get_variant(mesh));
    }

private:
    std::map<std::vector<long>, std::map<Tuple, ReturnTypeVariant>> m_data;
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

    using ReturnDataType = MultiMeshVisitorReturnData<NodeFunctor>;

    MultiMeshVisitor& visitor;


    ReturnDataType m_return_data;

    constexpr static bool HasEdgeFunctor = MMVisitor::HasEdgeFunctor;


    // checks whether return info data is the same as some mesh/simplex pair
    template <typename MeshType>
    bool is_same_as_final(const MeshType& mesh, const Simplex& s) const
    {
        return std::is_same_v<MeshType, ReturnMeshType> &&
               static_cast<const Mesh*>(&return_mesh) == static_cast<const Mesh*>(&mesh) &&
               return_simplex == s;
    }


    template <typename MeshType>
    void execute(MeshType& mesh, const simplex::Simplex& simplex) const
    {
        run(mesh, simplex);
    }


private:
    template <typename MeshType>
    auto run(MeshType& current_mesh, const simplex::Simplex& simplex) const
    {
        using CurReturnType = GetReturnType_t<MeshType>;

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
                auto chld_mesh_variant = wmtk::utils::as_mesh_variant(child_mesh_base);
                std::visit(
                    [&](const auto& child_mesh) {
                        using ChildMeshType = std::unwrap_ref_decay_t<decltype(child_mesh)>;
                        using ChildReturnType = GetReturnType_t<ChildMeshType>;


                        for (const simplex::Simplex& child_simplex : simplices) {
                            run_over_edge(current_mesh, current_return, child_mesh, child_simplex);
                        }
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
                auto chld_mesh_variant = wmtk::utils::as_mesh_variant(child_mesh_base);
                std::visit(
                    [&](const auto& child_mesh) {
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
