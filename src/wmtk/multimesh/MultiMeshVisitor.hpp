#pragma once
#include <spdlog/spdlog.h>
#include <type_traits>
#include <variant> //to get monostage
#include <wmtk/Mesh.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/mesh_type_from_primitive_type.hpp>


namespace wmtk::multimesh {


// if NodeFunctor returns a value then
template <typename NodeFunctor, typename EdgeFunctor = std::monostate>
class MultiMeshVisitor
{
public:
    constexpr static bool HasEdgeFunctor = !std::is_same_v<EdgeFunctor, std::monostate>;
    MultiMeshVisitor(NodeFunctor&& f, EdgeFunctor&& ef)
        : m_node_functor(f)
        , m_edge_functor(ef)
    {}
    MultiMeshVisitor(NodeFunctor&& f)
        : m_node_functor(f)
    {}

    // even if you try to use an interior mesh node this always just uses the root
    void execute_from_root(Mesh& mesh, const simplex::Simplex& simplex) const
    {
        if (!mesh.is_multi_mesh_root()) {
            Mesh& root = mesh.get_multi_mesh_root();
            const simplex::Simplex root_simplex = mesh.map_to_root(simplex);
            execute_mesh(root, root_simplex);
        } else {
            execute_mesh(mesh, simplex);
        }
    }
    // execute on teh subtree (if you want the entire tree use execute_from_root)
    void execute_mesh(Mesh& mesh, const simplex::Simplex& simplex) const
    {
        switch (mesh.top_simplex_type()) {
        case PrimitiveType::Vertex: {
            execute_PT<PrimitiveType::Vertex>(mesh, simplex);
            break;
        }
        case PrimitiveType::Edge: {
            execute_PT<PrimitiveType::Edge>(mesh, simplex);
            break;
        }
        case PrimitiveType::Face: {
            execute_PT<PrimitiveType::Face>(mesh, simplex);
            break;
        }
        case PrimitiveType::Tetrahedron: {
            execute_PT<PrimitiveType::Tetrahedron>(mesh, simplex);
            break;
        }
        }
    }

private:
    NodeFunctor m_node_functor;
    EdgeFunctor m_edge_functor;

private:
    template <PrimitiveType PT>
    static auto& as_primitive_type(Mesh& mesh)
    {
        // TODO: as other meshes become available use those!
        if constexpr (PT == PrimitiveType::Face) {
            return static_cast<wmtk::utils::mesh_type_from_primitive_type_t<PT>&>(mesh);
        } else {
            return mesh;
        }
    }
    template <PrimitiveType PT>
    auto execute_PT(Mesh& mesh, const simplex::Simplex& simplex) const
    {
        return execute_T(as_primitive_type<PT>(mesh), simplex);
    }
    template <typename MeshType>
    auto execute_T(MeshType& parent_mesh, const simplex::Simplex& simplex) const
    {
        using ParentReturnType =
            std::invoke_result_t<NodeFunctor, MeshType&, const simplex::Simplex&>;
        constexpr static bool NoReturn = std::is_void_v<ParentReturnType>;
        if constexpr (NoReturn) {
            m_node_functor(parent_mesh, simplex);
            for (auto& child_data : parent_mesh.m_multi_mesh_manager.children()) {
                Mesh& child_mesh = *child_data.mesh;
                auto simplices = parent_mesh.map_to_child(child_mesh, simplex);
                for (const simplex::Simplex& child_simplex : simplices) {
                    execute_mesh(child_mesh, child_simplex);
                }
            }
        } else {
            auto parent_return = m_node_functor(parent_mesh, simplex);
            for (auto& child_data : parent_mesh.m_multi_mesh_manager.children()) {
                Mesh& child_mesh = *child_data.mesh;
                auto simplices = parent_mesh.map_to_child(child_mesh, simplex);
                for (const simplex::Simplex& child_simplex : simplices) {
                    switch (child_mesh.top_simplex_type()) {
                    case PrimitiveType::Vertex: {
                        execute_mesh_pair_PT<PrimitiveType::Vertex>(
                            parent_mesh,
                            parent_return,
                            child_mesh,
                            child_simplex);
                        break;
                    }
                    case PrimitiveType::Edge: {
                        execute_mesh_pair_PT<PrimitiveType::Edge>(
                            parent_mesh,
                            parent_return,
                            child_mesh,
                            child_simplex);
                        break;
                    }
                    case PrimitiveType::Face: {
                        execute_mesh_pair_PT<PrimitiveType::Face>(
                            parent_mesh,
                            parent_return,
                            child_mesh,
                            child_simplex);
                        break;
                    }
                    case PrimitiveType::Tetrahedron: {
                        execute_mesh_pair_PT<PrimitiveType::Tetrahedron>(
                            parent_mesh,
                            parent_return,
                            child_mesh,
                            child_simplex);
                        break;
                    }
                    }
                }
            }
            return parent_return;
        }
    }


    template <PrimitiveType ChildPrimitive, typename ParentType, typename ParentReturn>
    auto execute_mesh_pair_PT(
        ParentType& parent_mesh,
        const ParentReturn& parent_return,
        Mesh& child_mesh,
        const simplex::Simplex& child_simplex) const
    {
        return execute_mesh_pair_T(
            parent_mesh,
            parent_return,
            as_primitive_type<ChildPrimitive>(child_mesh),
            child_simplex);
    }

    template <typename ParentType, typename ParentReturn, typename ChildMesh>
    void execute_mesh_pair_T(
        ParentType& parent,
        const ParentReturn& parent_return,
        ChildMesh& child_mesh,
        const simplex::Simplex& child_simplex) const
    {
        using ReturnType = std::invoke_result_t<NodeFunctor, ChildMesh&, const simplex::Simplex&>;
        constexpr static bool NoReturn = std::is_void_v<ReturnType>;

        if constexpr (NoReturn) {
            execute_T(child_mesh, child_simplex);
        } else {
            auto child_return = execute_T(child_mesh, child_simplex);
            spdlog::info("Has return data! should do edge functor {}", HasEdgeFunctor);

            if constexpr (HasEdgeFunctor) {
                m_edge_functor(parent, parent_return, child_mesh, child_return);
            }
        }
    }
};


template <typename NodeFunctor, typename EdgeFunctor>
MultiMeshVisitor(NodeFunctor&&, EdgeFunctor&&) -> MultiMeshVisitor<NodeFunctor, EdgeFunctor>;

template <typename NodeFunctor>
MultiMeshVisitor(NodeFunctor&&) -> MultiMeshVisitor<NodeFunctor, std::monostate>;

} // namespace wmtk::multimesh
