#include "VertexLaplacianSmooth.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/link.hpp>


namespace wmtk::operations {

VertexLaplacianSmooth::VertexLaplacianSmooth(const attribute::MeshAttributeHandle& handle)
    : m_attibute_handle(handle)
{
#if !defined(NDEBUG) // pragma to make sure this heldtype isn't useful
    using HeldType = attribute::MeshAttributeHandle::HeldType;
    assert(handle.held_type() == HeldType::Double);
#endif
}


bool VertexLaplacianSmooth::operator()(Mesh& mesh, const simplex::Simplex& simplex)
{
    const std::vector<simplex::Simplex> one_ring =
        simplex::link(mesh, simplex).simplex_vector(PrimitiveType::Vertex);

    std::visit(
        [&](const auto& handle) {
            using T = typename std::decay_t<decltype(handle)>::Type;
            using HeldType = attribute::MeshAttributeHandle::HeldType;
            constexpr static HeldType ht =
                attribute::MeshAttributeHandle::held_type_from_primitive<T>();

            auto run = [&](auto& accessor) {
                auto p_mid = accessor.vector_attribute(simplex.tuple());
                p_mid.setZero();
                for (const simplex::Simplex& s : one_ring) {
                    p_mid = p_mid + accessor.vector_attribute(s.tuple());
                }
                p_mid = p_mid / one_ring.size();
            };

            if constexpr (ht == HeldType::Double) {
                auto accessor = mesh.create_accessor<double>(handle);
                // run on double data
                run(accessor);
            }
        },
        m_attibute_handle.handle());

    return true;
}

} // namespace wmtk::operations
