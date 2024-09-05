#include "VertexLaplacianSmooth.hpp"
#include <wmtk/attribute/utils/HybridRationalAccessor.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/link.hpp>


namespace wmtk::operations {

VertexLaplacianSmooth::VertexLaplacianSmooth(const attribute::MeshAttributeHandle& handle)
    : m_attibute_handle(handle)
{
#if !defined(NDEBUG) // pragma to make sure this heldtype isn't useful
    using HeldType = attribute::MeshAttributeHandle::HeldType;
    assert(
        handle.held_type() == HeldType::Double || handle.held_type() == HeldType::HybridRational);
#endif
}


bool VertexLaplacianSmooth::operator()(Mesh& mesh, const simplex::Simplex& simplex)
{
    const std::vector<simplex::Simplex> one_ring =
        simplex::link(mesh, simplex).simplex_vector(PrimitiveType::Vertex);

    std::visit(
        [&](const auto& handle) {
            using T = typename std::decay_t<decltype(handle)>::value_type;
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
            } else if constexpr (ht == HeldType::HybridRational) {
                auto hybrid_accessor = attribute::utils::HybridRationalAccessor(mesh, handle);

                // identify if every simplex in hte neigbhorhood is rounded. If any simplex is not
                // yet rounded then we work with rationals
                bool use_rational = !hybrid_accessor.are_all_rounded(one_ring);
                if (use_rational) {
                    run(hybrid_accessor.get_rational_accessor());
                    hybrid_accessor.round(simplex.tuple(), false);
                } else {
                    // run(hybrid_accessor.get_double_accessor());
                    //  if they're all rounded then make sure that we set the result as rounded
                    // hybrid_accessor.lift(simplex.tuple(), true);
                }
            }
        },
        m_attibute_handle.handle());

    return true;
}

} // namespace wmtk::operations
