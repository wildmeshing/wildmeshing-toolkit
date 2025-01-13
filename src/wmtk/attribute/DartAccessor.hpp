#pragma once

#include <wmtk/autogen/SimplexAdjacency.hpp>
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/find_local_dart_action.hpp>
#include "Accessor.hpp"

namespace wmtk::attribute {
template <int Dim, typename MeshType>
class DartIndexAccessor
{
    static_assert(sizeof(int64_t) * (Dim + 1) >= sizeof(autogen::SimplexAdjacency<Dim>));
    using BaseAccessor = Accessor<int64_t, MeshType, Dim + 1>;

    DartIndexAccessor(BaseAccessor acc)
        : m_base_accessor{std::move(acc)}
    {}
    template <typename MeshType2>
    DartIndexAccessor(const DartIndexAccessor<Dim, MeshType2>& o)
        : m_base_accessor{o.m_base_accessor}
    {}

    autogen::SimplexAdjacency<Dim>& operator[](int64_t index)
    {
        return reinterpret_cast<autogen::SimplexAdjacency<Dim>&>(
            m_base_accessor.index_access().vector_attribute(index));
    }
    const autogen::SimplexAdjacency<Dim>& operator[](int64_t index) const
    {
        return reinterpret_cast<const autogen::SimplexAdjacency<Dim>&>(
            m_base_accessor.index_access().const_vector_attribute(index));
    }

    const MeshType& mesh() const { return m_base_accessor.mesh(); }

    autogen::Dart switch_facet(int64_t global_id, int8_t local_orientation)
    {
        PrimitiveType FT = mesh().top_simplex_type();
        PrimitiveType BT = FT - 1;
        const autogen::SimplexDart& sd = autogen::SimplexDart::get_singleton(FT);
        const auto anchor = (*this)[global_id][local_orientation];
        const auto dual_anchor = (*this)[anchor.global_id()][anchor.local_orientation()];
        // anchor = A * dual_anchor

        const int8_t act = autogen::find_local_dart_action(
            sd,
            dual_anchor.local_orientation(),
            anchor.global_orientation());

        return autogen::Dart(anchor.global_id(), sd.product(act, local_orientation));
    }

protected:
    BaseAccessor m_base_accessor;
};

template <int Dim, typename MeshType>
class DartAccessor : public DartIndexAccessor<Dim, MeshType>
{
    using IndexBaseType = DartIndexAccessor<Dim, MeshType>;

    using IndexBaseType::mesh;

    template <typename T>
    autogen::SimplexAdjacency<Dim>& operator[](const T& t)
    {
        return reinterpret_cast<autogen::SimplexAdjacency<Dim>&>(
            IndexBaseType::m_base_accessor.vector_attribute(index));
    }
    template <typename T>
    const autogen::SimplexAdjacency<Dim>& operator[](const T& t) const
    {
        return reinterpret_cast<const autogen::SimplexAdjacency<Dim>&>(
            IndexBaseType::m_base_accessor.const_vector_attribute(t));
    }
    static wmtk::attribute::TypedAttributeHandle<int64_t>
    register_attribute(MeshType& m, const std::string_view& name, bool do_populate = false)
    {
        auto handle = m.template register_attribute_typed<int64_t>(
            std::string(name),
            m.top_simplex_type() - 1,
            Dim + 1,
            false,
            -1);
        if (do_populate) {
            DartAccessor acc(BaseAccessor(m, handle));
            acc.populate();
        }
    }
    void populate()
    {
        PrimitiveType FT = mesh().top_simplex_type();
        PrimitiveType BT = FT - 1;
        const autogen::SimplexDart& sd = autogen::SimplexDart::get_singleton(FT);
        for (Tuple t : mesh().get_all(BT)) {
            autogen::Dart d = sd.dart_from_tuple(t);
            int8_t local_index = sd.simplex_index(d, BT);
            if (mesh().is_boundary(t)) {
                (*this)[d][local_index] = autogen::Dart();
            } else {
                Tuple ot = mesh().switch_tuple(t, FT);
                autogen::Dart od = sd.dart_from_tuple(t);
                int8_t other_local_index = sd.simplex_index(od, BT);
                (*this)[d][local_index] = od;
                (*this)[od][other_local_index] = d;
            }
        }
    }

    autogen::Dart switch_facet(const autogen::Dart& d)
    {
        return IndexBaseType::switch_facet(d.global_id(), d.local_orientation());
    }
    autogen::Dart switch_facet(const autogen::DartWrap& d)
    {
        return IndexBaseType::switch_facet(d.global_id(), d.local_orientation());
    }
};
} // namespace wmtk::attribute
