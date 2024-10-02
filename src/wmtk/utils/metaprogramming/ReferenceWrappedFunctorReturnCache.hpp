#pragma once
#include <algorithm>
#include <functional>
#include <map>
#include <stdexcept>
#include <tuple>
#include <variant>

#include "ReferenceWrappedFunctorReturnType.hpp"
namespace wmtk::utils::metaprogramming {

namespace detail {
class DefaultComparatorType
{
public:
    class Equal
    {
    public:
        template <typename T>
        bool operator()(const T& a, const T& b) const
        {
            return std::equal_to<T>{}(a, b);
        }
    };
    class Less
    {
    public:
        template <typename T>
        bool operator()(const T& a, const T& b) const
        {
            return std::less<T>{}(a, b);
        }
    };
};
// Interface for reading off the return values from data
template <
    typename Functor,
    typename BaseVariantTraitsType,
    typename ComparatorType,
    typename... OtherArgumentTypes>
class ReferenceWrappedFunctorReturnCache
{
public:
    using TypeHelper = detail::ReferenceWrappedFunctorReturnType<
        Functor,
        typename BaseVariantTraitsType::AllReferenceTuple,
        OtherArgumentTypes...>;
    static_assert(!TypeHelper::all_void);
    using ReturnVariant = typename TypeHelper::type;

    using RefVariantType = typename BaseVariantTraitsType::ReferenceVariant;
    using BaseType = typename BaseVariantTraitsType::BaseType;
    using TupleType = typename BaseVariantTraitsType::DerivedTypesTuple;
    using RefTupleType = typename BaseVariantTraitsType::ReferenceTuple;

    // Add new data by giving the InputType
    // InputType is used to make sure the pair of Input/Output is valid and to
    // extract an id
    // NOTE this passes value then key because the key can have variable arguments :(
    template <typename InputType, typename ReturnType>
    void add(ReturnType&& return_data, const InputType& input, const OtherArgumentTypes&... args)
    {
        using ReturnType_t = std::decay_t<ReturnType>;
        if constexpr (std::is_same_v<ReturnType_t, void>) {
            return;
        }
        static_assert(
            !std::is_same_v<std::decay_t<InputType>, BaseType>,
            "Don't pass in a input, use variant/visitor to get its "
            "derived type");
        // static_assert(
        //     !std::
        //         holds_alternative<std::reference_wrapper<std::decay_t<InputType>>,
        //         RefVariantType>,
        //     "the input type must be seen in the set of valid input variants");
        //  if the user passed in a input class lets try re-invoking with a
        //  derived type
        auto id = get_id(input, args...);
        using ExpectedReturnType = typename TypeHelper::template ReturnType<InputType>;

        static_assert(
            std::is_convertible_v<ReturnType_t, ExpectedReturnType>,
            "Second argument should be the return value of a Functor "
            "(or convertible at "
            "least) ");

        auto [it, did_insert] = m_data.try_emplace(
            id,
            ReturnVariant(
                std::in_place_type_t<ExpectedReturnType>{},
                std::forward<ReturnType>(return_data)));
        if (!did_insert && m_enable_overwrites) {
            throw std::runtime_error(
                "Tried to overwite a value already stored in the return value cache");
        }
    }


    // get the type specific input
    template <typename InputType>
    auto get(const InputType& input, const OtherArgumentTypes&... ts) const ->
        typename TypeHelper::template ReturnTypeConstRef<InputType>
    {
        static_assert(
            !std::is_same_v<std::decay_t<InputType>, BaseType>,
            "Don't pass in a input, use variant/visitor to get its "
            "derived type");
        using ExpectedReturnType = typename TypeHelper::template ReturnType<InputType>;
        if constexpr (std::is_same_v<ExpectedReturnType, void>) {
            return;
        }

        return std::get<ExpectedReturnType>(get_variant(input, ts...));
    }
    // get the type specific input
    // template <typename InputType>
    // auto get(const InputType& input, const OtherArgumentTypes&... ts) ->
    //    typename TypeHelper::template ReturnTypeRef<InputType>
    //{
    //    static_assert(
    //        !std::is_same_v<std::decay_t<InputType>, BaseType>,
    //        "Don't pass in a input, use variant/visitor to get its "
    //        "derived type");
    //    using ExpectedReturnType = typename TypeHelper::template ReturnType<InputType>;
    //    if constexpr (std::is_same_v<ExpectedReturnType, void>) {
    //        return;
    //    }

    //    return std::get<ExpectedReturnType>(get_variant(input, ts...));
    //}

    // let user get the variant for a specific Input derivate
    const auto& get_variant(const BaseType& input, const OtherArgumentTypes&... ts) const
    {
        auto id = get_id(input, ts...);
        return m_data.at(id);
    }

    bool has_variant(const BaseType& input, const OtherArgumentTypes&... ts) const
    {
        auto id = get_id(input, ts...);
        return m_data.find(id) != m_data.end();
    }

    // a pointer to an input and some other arguments
    using KeyType = std::tuple<const BaseType*, OtherArgumentTypes...>;

    std::vector<KeyType> keys() const
    {
        std::vector<KeyType> ret;
        ret.reserve(m_data.size());
        std::transform(m_data.begin(), m_data.end(), std::back_inserter(ret), [](const auto& pr) {
            return pr.first;
        });
        return ret;
    }

    auto get_id(const BaseType& input, const OtherArgumentTypes&... ts) const
    {
        // other applications might use a fancier version of get_id
        return KeyType(&input, ts...);
    }

    // let user get the variant for a specific Input derivate
    const auto& get_variant(const KeyType& key) const { return m_data.at(key); }


    auto begin() const { return m_data.begin(); }
    auto end() const { return m_data.end(); }

    void set_enable_overwrites(bool value) { m_enable_overwrites = value; }

private:
    std::map<KeyType, ReturnVariant, typename ComparatorType::Less> m_data;
    bool m_enable_overwrites = false;
};

} // namespace detail

template <typename Functor, typename BaseVariantTraitsType, typename... OtherArgumentTypes>
constexpr static bool all_return_void_v = detail::ReferenceWrappedFunctorReturnType<
    Functor,
    typename BaseVariantTraitsType::AllReferenceTuple,
    OtherArgumentTypes...>::all_void;


// returns void if everything returns void

template <
    typename Functor,
    typename BaseVariantTraitsType,
    typename ComparatorType,
    typename... OtherArgumentTypes>
using ReferenceWrappedFunctorReturnCacheCustomComparator = std::conditional_t<
    all_return_void_v<Functor, BaseVariantTraitsType, OtherArgumentTypes...>,
    std::monostate,
    detail::ReferenceWrappedFunctorReturnCache<
        Functor,
        BaseVariantTraitsType,
        ComparatorType,
        OtherArgumentTypes...>>;
template <typename Functor, typename BaseVariantTraitsType, typename... OtherArgumentTypes>
using ReferenceWrappedFunctorReturnCache = std::conditional_t<
    all_return_void_v<Functor, BaseVariantTraitsType, OtherArgumentTypes...>,
    std::monostate,
    detail::ReferenceWrappedFunctorReturnCache<
        Functor,
        BaseVariantTraitsType,
        detail::DefaultComparatorType,
        OtherArgumentTypes...>>;
} // namespace wmtk::utils::metaprogramming
