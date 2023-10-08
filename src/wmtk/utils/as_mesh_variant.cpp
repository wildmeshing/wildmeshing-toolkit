#include "as_mesh_variant.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/mesh_type_from_primitive_type.hpp>


namespace wmtk::utils {

namespace {
template <PrimitiveType PT>
using as_reference_wrapper_t =
    std::reference_wrapper<wmtk::utils::mesh_type_from_primitive_type_t<PT>>;
}

template <PrimitiveType PT>
auto as_ref(Mesh& m)
{
    return std::reference_wrapper(
        static_cast<wmtk::utils::mesh_type_from_primitive_type_t<PT>&>(m));
}
template <PrimitiveType PT>
auto as_cref(const Mesh& m)
{
    return std::reference_wrapper(
        static_cast<const wmtk::utils::mesh_type_from_primitive_type_t<PT>&>(m));
}

MeshVariantType as_mesh_variant(Mesh& mesh)
{
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Vertex: {
        return std::reference_wrapper(mesh);
        // return as_ref<PrimitiveType::Vertex>(mesh);
    }
    case PrimitiveType::Edge: {
        return std::reference_wrapper(mesh);
        // return as_ref<PrimitiveType::Edge>(mesh);
    }
    case PrimitiveType::Face: {
        return as_ref<PrimitiveType::Face>(mesh);
    }
    case PrimitiveType::Tetrahedron: {
        return as_ref<PrimitiveType::Tetrahedron>(mesh);
    }
    default: return std::reference_wrapper(mesh);
    }
}
ConstMeshVariantType as_const_mesh_variant(const Mesh& mesh)
{
    switch (mesh.top_simplex_type()) {
    case PrimitiveType::Vertex: {
        return std::reference_wrapper(mesh);
        // return as_cref<PrimitiveType::Vertex>(mesh);
    }
    case PrimitiveType::Edge: {
        return std::reference_wrapper(mesh);
        // return as_cref<PrimitiveType::Edge>(mesh);
    }
    case PrimitiveType::Face: {
        return as_cref<PrimitiveType::Face>(mesh);
    }
    case PrimitiveType::Tetrahedron: {
        return as_cref<PrimitiveType::Tetrahedron>(mesh);
    }
    default: return std::reference_wrapper(mesh);
    }
}
} // namespace wmtk::utils
