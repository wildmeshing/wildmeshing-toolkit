#pragma once

#include <spdlog/spdlog.h>
#include <wmtk/autogen/Dart.hpp>
#include <wmtk/autogen/SimplexAdjacency.hpp>
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/find_local_dart_action.hpp>
#include "Accessor.hpp"

namespace wmtk {
class PointMesh;
class EdgeMesh;
class TriMesh;
class TetMesh;
} // namespace wmtk
namespace wmtk::attribute {
template <int Dim, typename MeshType>
class DartIndexAccessor
{
public:
    static_assert(sizeof(int64_t) * (Dim + 1) >= sizeof(autogen::SimplexAdjacency<Dim>));
    using BaseAccessor = Accessor<int64_t, MeshType, Dim + 1>;

    DartIndexAccessor(BaseAccessor acc)
        : m_base_accessor{std::move(acc)}
    {}

    DartIndexAccessor(const MeshType& mesh, const MeshAttributeHandle& mah)
        : m_base_accessor{mesh.template create_const_accessor<int64_t, Dim + 1>(mah)}
    {}
    DartIndexAccessor(const MeshType& mesh, const TypedAttributeHandle<int64_t>& h)
        : m_base_accessor{mesh.template create_const_accessor<int64_t, Dim + 1>(h)}
    {}
    template <typename MeshType2>
    DartIndexAccessor(const DartIndexAccessor<Dim, MeshType2>& o)
        : m_base_accessor{o.m_base_accessor}
    {}

    autogen::SimplexAdjacency<Dim>& operator[](int64_t index)
    {
        return *reinterpret_cast<autogen::SimplexAdjacency<Dim>*>(
            m_base_accessor.index_access().vector_attribute(index).data());
    }
    const autogen::SimplexAdjacency<Dim>& operator[](int64_t index) const
    {
        return *reinterpret_cast<const autogen::SimplexAdjacency<Dim>*>(
            m_base_accessor.index_access().const_vector_attribute(index).data());
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
            anchor.local_orientation());

        return autogen::Dart(anchor.global_id(), sd.product(act, local_orientation));
    }

protected:
    BaseAccessor m_base_accessor;
};

template <int Dim, typename MeshType>
class DartAccessor : public DartIndexAccessor<Dim, MeshType>
{
public:
    using IndexBaseType = DartIndexAccessor<Dim, MeshType>;

protected:
    using IndexBaseType::m_base_accessor;

public:
    using IndexBaseType::IndexBaseType;

    using IndexBaseType::mesh;

    autogen::SimplexAdjacency<Dim>& operator[](const autogen::Dart& t)
    {
        return IndexBaseType::operator[](m_base_accessor.index(t));
    }
    const autogen::SimplexAdjacency<Dim>& operator[](const autogen::Dart& t) const
    {
        return IndexBaseType::operator[](m_base_accessor.index(t));
    }
    static wmtk::attribute::TypedAttributeHandle<int64_t>
    register_attribute(MeshType& m, const std::string_view& name, bool do_populate = false)
    {
        spdlog::info("Starting to register");
        auto handle = m.template register_attribute_typed<int64_t>(
            std::string(name),
            m.top_simplex_type() - 1,
            Dim + 1,
            false,
            -1);
        if (do_populate) {
            spdlog::info("Creating accessor");
            DartAccessor acc(m, handle);
            spdlog::info("Starting to populate");
            acc.populate();
        }
        spdlog::info("Returning handle");
        return handle;
    }

    void fuse(const autogen::Dart& d, const autogen::Dart& od)
    {
        const PrimitiveType FT = mesh().top_simplex_type();
        const PrimitiveType BT = FT - 1;
        const autogen::SimplexDart& sd = autogen::SimplexDart::get_singleton(FT);
        int8_t local_index = sd.simplex_index(d, BT);
        int8_t other_local_index = sd.simplex_index(od, BT);
        autogen::SimplexAdjacency<Dim>& sad = (*this)[d];
        autogen::SimplexAdjacency<Dim>& saod = (*this)[od];
        sad[local_index] = od;
        saod[other_local_index] = d;
    }
    void populate()
    {
        PrimitiveType FT = mesh().top_simplex_type();
        PrimitiveType BT = FT - 1;
        const autogen::SimplexDart& sd = autogen::SimplexDart::get_singleton(FT);
        for (Tuple t : mesh().get_all(BT)) {
            autogen::Dart d = sd.dart_from_tuple(t);
            if (mesh().is_boundary(BT, t)) {
                int8_t local_index = sd.simplex_index(d, BT);
                autogen::SimplexAdjacency<Dim>& sad = (*this)[d];
                sad[local_index] = autogen::Dart();
            } else {
                Tuple ot = mesh().switch_tuple(t, FT);
                autogen::Dart od = sd.dart_from_tuple(t);
                fuse(d, od);
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

    autogen::Dart switch_dart(const autogen::Dart& d, PrimitiveType pt)
    {
        const PrimitiveType FT = mesh().top_simplex_type();
        if (pt == FT) {
            return switch_facet(d);
        } else {
            const autogen::SimplexDart& sd = autogen::SimplexDart::get_singleton(FT);
            return sd.act(d, sd.primitive_as_index(pt));
        }
    }
};


template <typename Handle>
DartAccessor(const PointMesh& p, const Handle&) -> DartAccessor<2, PointMesh>;
template <typename Handle>
DartAccessor(const EdgeMesh&, const Handle&) -> DartAccessor<3, EdgeMesh>;
template <typename Handle>
DartAccessor(const TriMesh&, const Handle&) -> DartAccessor<4, TriMesh>;
template <typename Handle>
DartAccessor(const TetMesh&, const Handle&) -> DartAccessor<5, TetMesh>;

template <typename MeshType>
auto register_dart_attribute(MeshType& mesh, const std::string_view& name, bool do_populate = false)
{
    return decltype(DartAccessor(
        mesh,
        std::declval<MeshAttributeHandle>()))::register_attribute(mesh, name, do_populate);
}

} // namespace wmtk::attribute
