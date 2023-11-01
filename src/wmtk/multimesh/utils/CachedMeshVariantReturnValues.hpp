#pragma once
#include <map>

#include <wmtk/utils/metaprogramming/MeshVariantTraits.hpp>
#include <wmtk/utils/metaprogramming/unwrap_ref.hpp>


namespace wmtk::multimesh::utils {


// A helper class for specifying per-type return types from an input functor
// Assumes the argument is the variant type being selected form, all other
// arguments are passed in as const references
template <typename Functor, typename... Ts>
struct ReturnVariantHelper
{
};
template <typename Functor, typename... VTs, typename... Ts>
struct ReturnVariantHelper<Functor, std::variant<VTs...>, Ts...>
{
    // For a specific type in the variant, get the return type
    template <typename T>
    using ReturnType = std::decay_t<std::invoke_result_t<
        Functor,
        wmtk::utils::metaprogramming::unwrap_ref_decay_t<T>&,
        const Ts&...>>;

    template <typename T>
    using ReturnTypeConst = std::decay_t<std::invoke_result_t<
        Functor,
        const wmtk::utils::metaprogramming::unwrap_ref_decay_t<T>&,
        const Ts&...>>;

    // check what happens if we return a const ref or non-const ref
    template <bool IsConst, typename T>
    using ReturnType_const = std::conditional_t<IsConst, ReturnTypeConst<T>, ReturnType<T>>;

    // Get an overall variant for the types
    using type = std::variant<ReturnType<VTs>...>;
    using const_type = std::variant<ReturnTypeConst<VTs>...>;
    template <bool IsConst>
    using type_const = std::variant<ReturnType_const<IsConst, VTs>...>;
};

// Interface for reading off the return values from data
template <typename Functor, typename... OtherArgumentTypes>
class CachedMeshVariantReturnValues
{
public:
    using MeshVariantTraits = wmtk::utils::metaprogramming::MeshVariantTraits;
    using MeshVariantType = MeshVariantTraits::ReferenceVariant;
    using ConstMeshVariantType = MeshVariantTraits::ConstReferenceVariant;

    using TypeHelper = ReturnVariantHelper<Functor, MeshVariantType, OtherArgumentTypes...>;
    using ReturnVariant = typename TypeHelper::type;

    // a pointer to an input and some other arguments
    using KeyType = std::tuple<const Mesh*, OtherArgumentTypes...>;

    auto get_id(const Mesh& input, const OtherArgumentTypes&... ts) const
    {
        // other applications might use a fancier version of get_id
        return KeyType(&input, ts...);
    }

    // Add new data by giving the MeshType
    // MeshType is used to make sure the pair of Mesh/Output is valid and to
    // extract an id
    template <typename MeshType, typename ReturnType>
    void add(const MeshType& input, ReturnType&& return_data, const OtherArgumentTypes&... args)
    {
        using ReturnType_t = std::decay_t<ReturnType>;
        static_assert(
            !std::is_same_v<std::decay_t<MeshType>, Mesh>,
            "Don't pass in a input, use variant/visitor to get its "
            "derived type");
        // if the user passed in a input class lets try re-invoking with a
        // derived type
        auto id = get_id(input, args...);
        using ExpectedReturnType = typename TypeHelper::template ReturnType<MeshType>;

        static_assert(
            std::is_convertible_v<ReturnType_t, ExpectedReturnType>,
            "Second argument should be the return value of a Functor "
            "(or convertible at "
            "least) ");

        m_data.emplace(
            id,
            ReturnVariant(
                std::in_place_type_t<ExpectedReturnType>{},
                std::forward<ReturnType>(return_data)));
    }

    // let user get the variant for a specific Mesh derivate
    const auto& get_variant(const Mesh& input, const OtherArgumentTypes&... ts) const
    {
        auto id = get_id(input, ts...);
        return m_data.at(id);
    }

    // get the type specific input
    template <typename MeshType>
    auto get(const MeshType& input, const OtherArgumentTypes&... ts) const
    {
        static_assert(
            !std::is_same_v<std::decay_t<MeshType>, Mesh>,
            "Don't pass in a input, use variant/visitor to get its "
            "derived type");
        using ExpectedReturnType = typename TypeHelper::template ReturnType<MeshType>;

        return std::get<ExpectedReturnType>(get_variant(input, ts...));
    }

private:
    std::map<KeyType, ReturnVariant> m_data;
};


template <typename Functor>
CachedMeshVariantReturnValues(Functor&& f) -> CachedMeshVariantReturnValues<std::decay_t<Functor>>;


} // namespace wmtk::multimesh::utils
