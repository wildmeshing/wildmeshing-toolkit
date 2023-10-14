#pragma once
#include <map>
#include <tuple>

#include "ReferenceWrappedFunctorReturnType.hpp"
namespace wmtk::utils::metaprogramming {

// Interface for reading off the return values from data
template <typename Functor, typename BaseVariantTraitsType, typename... OtherArgumentTypes>
class ReferenceWrappedFunctorReturnCache
{
public:
    using TypeHelper =
        ReferenceWrappedFunctorReturnType<Functor, BaseVariantTraitsType, OtherArgumentTypes...>;
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

        m_data.emplace(
            id,
            ReturnVariant(
                std::in_place_type_t<ExpectedReturnType>{},
                std::forward<ReturnType>(return_data)));
    }


    // get the type specific input
    template <typename InputType>
    auto get(const InputType& input, const OtherArgumentTypes&... ts) const
    {
        static_assert(
            !std::is_same_v<std::decay_t<InputType>, BaseType>,
            "Don't pass in a input, use variant/visitor to get its "
            "derived type");
        using ExpectedReturnType = typename TypeHelper::template ReturnType<InputType>;

        return std::get<ExpectedReturnType>(get_variant(input, ts...));
    }

    // let user get the variant for a specific Input derivate
    const auto& get_variant(const BaseType& input, const OtherArgumentTypes&... ts) const
    {
        auto id = get_id(input, ts...);
        return m_data.at(id);
    }

private:
    // a pointer to an input and some other arguments
    using KeyType = std::tuple<const BaseType*, OtherArgumentTypes...>;

    auto get_id(const BaseType& input, const OtherArgumentTypes&... ts) const
    {
        // other applications might use a fancier version of get_id
        return KeyType(&input, ts...);
    }

    std::map<KeyType, ReturnVariant> m_data;
};

} // namespace wmtk::utils::metaprogramming
