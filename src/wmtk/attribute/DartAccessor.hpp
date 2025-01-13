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

    autogen::Dart switch_facet(int64_t global_id, int8_t local_orientation) const
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

    template <typename IT, typename OT>
    autogen::SimplexAdjacency<Dim>& operator[](const autogen::_Dart<IT, OT>& t)
    {
        return IndexBaseType::operator[](m_base_accessor.index(t));
    }
    template <typename IT, typename OT>
    const autogen::SimplexAdjacency<Dim>& operator[](const autogen::_Dart<IT, OT>& t) const
    {
        return IndexBaseType::operator[](m_base_accessor.index(t));
    }

    template <typename IT, typename OT>
    int8_t get_boundary_local_index(const autogen::_Dart<IT, OT>& d) const
    {
        const PrimitiveType FT = mesh().top_simplex_type();
        const PrimitiveType BT = FT - 1;
        const autogen::SimplexDart& sd = autogen::SimplexDart::get_singleton(FT);
        return sd.simplex_index(d, BT);
    }
    template <typename IT, typename OT>
    autogen::Dart get_neighbor(const autogen::_Dart<IT, OT>& d) const
    {
        const auto& sa = (*this)[d];
        return sa[get_boundary_local_index(d)];
    }
    template <typename IT, typename OT>
    autogen::DartWrap get_neighbor(const autogen::_Dart<IT, OT>& d)
    {
        const auto& sa = (*this)[d];
        return sa[get_boundary_local_index(d)];
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
        autogen::SimplexAdjacency<Dim>& sad = (*this)[d];
        autogen::SimplexAdjacency<Dim>& saod = (*this)[od];
        sad[get_boundary_local_index(d)] = od;
        saod[get_boundary_local_index(od)] = d;
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
                assert(local_index == get_boundary_local_index(d));
                autogen::SimplexAdjacency<Dim>& sad = (*this)[d];
                sad[local_index] = autogen::Dart();
            } else {
                Tuple ot = mesh().switch_tuple(t, FT);
                autogen::Dart od = sd.dart_from_tuple(t);
                fuse(d, od);
            }
        }
    }

    template <typename IT, typename OT>
    autogen::Dart switch_facet(const autogen::_Dart<IT, OT>& d) const
    {
        return IndexBaseType::switch_facet(d.global_id(), d.local_orientation());
    }

    template <typename IT, typename OT>
    autogen::Dart switch_dart(const autogen::_Dart<IT, OT>& d, PrimitiveType pt) const
    {
        const PrimitiveType FT = mesh().top_simplex_type();
        if (pt == FT) {
            return switch_facet(d);
        } else {
            const autogen::SimplexDart& sd = autogen::SimplexDart::get_singleton(FT);
            autogen::Dart ret = sd.act(d, sd.primitive_as_index(pt));
            return ret;
        }
    }
    template <typename IT, typename OT>
    bool is_boundary(const autogen::_Dart<IT, OT>& d) const
    {
        return get_neighbor(d).is_null();
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
