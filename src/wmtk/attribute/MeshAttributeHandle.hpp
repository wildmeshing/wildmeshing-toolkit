#pragma once
#include <wmtk/utils/Rational.hpp>
//
#include "TypedAttributeHandle.hpp"
#include "utils/HybridRationalAttribute.hpp"

#include <variant>

namespace wmtk {
class Mesh;
} // namespace wmtk

namespace wmtk::attribute {

/* @brief Handle that can construct an accessor on its own
 * NOTE: This naming is inconsistent with the existing
 * AttributeHandle/MeshAttributeHandle nomenclature, but in the future most
 * applications should store MeshAttributeHandles instead of
 * MeshAttributeHandle, and after most of those changes are made we will
 * deprecate that name.
 */
class MeshAttributeHandle
{
public:
    using HandleVariant = std::variant<
        TypedAttributeHandle<char>,
        TypedAttributeHandle<int64_t>,
        TypedAttributeHandle<double>,
        TypedAttributeHandle<wmtk::Rational>,
        utils::HybridRationalAttribute<Eigen::Dynamic>>;

    using ValueVariant = std::variant<char, int64_t, double, wmtk::Rational>;

    enum class HeldType { Char = 0, Int64 = 1, Double = 2, Rational = 3, HybridRational = 4 };

    template <HeldType Type>
    using held_handle_type = std::variant_alternative<size_t(Type), HandleVariant>;

    template <HeldType Type>
    using held_primitive_type = typename held_handle_type<Type>::Type;
    template <typename T>
    constexpr static HeldType held_type_from_primitive();

    template <typename T>
    constexpr static bool attribute_type_is_basic();

    friend class wmtk::Mesh;
    friend class wmtk::hash<MeshAttributeHandle>;
    MeshAttributeHandle() = default;
    MeshAttributeHandle(Mesh& m, const HandleVariant& h);
    MeshAttributeHandle(const MeshAttributeHandle& o) = default;
    MeshAttributeHandle(MeshAttributeHandle&& o) = default;
    MeshAttributeHandle& operator=(const MeshAttributeHandle& o) = default;
    MeshAttributeHandle& operator=(MeshAttributeHandle&& o) = default;

    bool operator==(const MeshAttributeHandle& o) const
    {
        return m_handle == o.m_handle && m_mesh == o.m_mesh;
    }


    // reutrns if the target mesh is the same as the one represented in the handle
    bool is_same_mesh(const Mesh&) const;


    // returns if this handle was initialized
    bool is_valid() const;

    PrimitiveType primitive_type() const;
    template <typename T>
    PrimitiveType primitive_typeT() const;
    // AttributeHandle base_handle() const ;


    template <typename T>
    const TypedAttributeHandle<T>& as() const;

    template <HeldType Type>
    auto as_from_held_type() const -> const TypedAttributeHandle<held_primitive_type<Type>>&;
    // returns if the held attribute uses the primitive T


    // returns if the held attribute uses the primitive T
    template <typename T>
    bool holds() const;

    // holds basic type
    bool holds_basic_type() const;

    // returns if the held attribute uses the held type primitive Type
    template <HeldType Type>
    bool holds_from_held_type() const;

    HeldType held_type() const;

    Mesh& mesh();
    const Mesh& mesh() const;

    HandleVariant& handle() { return m_handle; }
    const HandleVariant& handle() const { return m_handle; }
    //// creates mutable accessors
    //// Implementations are in the MutableAccessor.hpp
    //// for historical reasons note that the following two classes are the same:
    //// wmtk::attribute::MutableAccessor
    //// wmtk::Accessor
    // MutableAccessor<T> create_accessor();

    //// Creates const accessors
    //// Implementations are in the ConstAccessor.hpp
    //// for historical reasons note that the following two classes are the same:
    //// wmtk::attribute::ConstAccessor
    //// wmtk::ConstAccessor
    // ConstAccessor<T> create_const_accessor() const;
    // ConstAccessor<T> create_accessor() const;

    // return the dimension of the attribute (i.e the number of values stored per simplex)
    int64_t dimension() const;

    // std::string name() const;


private:
    Mesh* m_mesh = nullptr;
    HandleVariant m_handle;
};

template <typename T>
inline const TypedAttributeHandle<T>& MeshAttributeHandle::as() const
{
    return std::get<TypedAttributeHandle<T>>(m_handle);
}


template <typename T>
inline bool MeshAttributeHandle::holds() const
{
    return std::holds_alternative<TypedAttributeHandle<T>>(m_handle);
}

inline bool MeshAttributeHandle::holds_basic_type() const
{
    return std::visit(
        [](const auto& h) -> bool { return attribute_type_is_basic<std::decay_t<decltype(h)>>(); },
        m_handle);
}
template <MeshAttributeHandle::HeldType Type>
inline auto MeshAttributeHandle::as_from_held_type() const
    -> const TypedAttributeHandle<held_primitive_type<Type>>&
{
    return as<held_primitive_type<Type>>();
}

template <MeshAttributeHandle::HeldType Type>
inline bool MeshAttributeHandle::holds_from_held_type() const
{
    return MeshAttributeHandle::holds<held_primitive_type<Type>>();
}

template <typename T>
inline constexpr auto MeshAttributeHandle::held_type_from_primitive() -> HeldType
{
    if constexpr (std::is_same_v<T, char>) {
        return HeldType::Char;
    }
    if constexpr (std::is_same_v<T, double>) {
        return HeldType::Double;
    }
    if constexpr (std::is_same_v<T, int64_t>) {
        return HeldType::Int64;
    }
    if constexpr (std::is_same_v<T, wmtk::Rational>) {
        return HeldType::Rational;
    }
    if constexpr (std::is_same_v<T, utils::HybridRationalAttribute<Eigen::Dynamic>>) {
        return HeldType::HybridRational;
    }
    return HeldType::Char;
}

inline PrimitiveType MeshAttributeHandle::primitive_type() const
{
    return std::visit([](const auto& h) { return h.primitive_type(); }, m_handle);
}
template <typename T>
inline PrimitiveType MeshAttributeHandle::primitive_typeT() const
{
    return std::get<T>(m_handle).primitive_type();
}

template <typename T>
constexpr inline bool MeshAttributeHandle::attribute_type_is_basic()
{
    return //
        std::is_same_v<T, double> || std::is_same_v<T, int64_t> || std::is_same_v<T, char> ||
        std::is_same_v<T, wmtk::Rational>;
}
} // namespace wmtk::attribute
