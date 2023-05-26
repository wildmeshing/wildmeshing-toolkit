#include <numeric>
//
#include <highfive/H5DataType.hpp>
#include <highfive/H5File.hpp>

namespace wmtk::serialization {
namespace detail {
// template <typename T>
// struct StlHdf5Serialization;
//
//} // namespace detail
//
// template <typename T>
// HighFive::DataType get_stl_hdf5_serialization()
//{
//    return detail::StlHdf5Serialization<T>::datatype();
//}
//
//
// template <typename... Types, StlType>
// struct GetStlType
//{
//    using StlType = StlType_;
//};
//
//
class UserStlConverterBase
{
    virtual void to_stl() = 0;
    virtual void from_stl() = 0;
};
template <typename UserType>
struct DefaultUserToStlConverterStlType
{
};
template <>
struct DefaultUserToStlConverterStlType<int>
{
    using type = int;
};
template <>
struct DefaultUserToStlConverterStlType<char>
{
    using type = char;
};
template <>
struct DefaultUserToStlConverterStlType<float>
{
    using type = float;
};
template <>
struct DefaultUserToStlConverterStlType<double>
{
    using type = double;
};

template <typename T, size_t S>
struct DefaultUserToStlConverterStlType<std::array<T, S>>
{
    using type = std::array<typename DefaultUserToStlConverterStlType<T>::type, S>;
};

template <typename... T>
struct DefaultUserToStlConverterStlType<std::tuple<T...>>
{
    using type = std::tuple<typename DefaultUserToStlConverterStlType<T>::type...>;
};

template <typename A, typename B>
struct DefaultUserToStlConverterStlType<std::pair<A, B>>
{
    using type = std::pair<
        typename DefaultUserToStlConverterStlType<A>::type,
        typename DefaultUserToStlConverterStlType<B>::type>;
};

template <typename T, int R>
struct DefaultUserToStlConverterStlType<Eigen::Matrix<T, R, 1>>
{
    using type = std::array<T, size_t(R)>;
};

template <typename UserType>
using DefaultUserToStlConverterStlType_t =
    typename DefaultUserToStlConverterStlType<UserType>::type;

template <typename UserType_, typename StlType_ = DefaultUserToStlConverterStlType_t<UserType_>>
struct DefaultSingleConverter
{
    using StlType = StlType_;
    using UserType = UserType_;
    // adding some constraints for calling this default implementation
    constexpr static bool IsArithmetic =
        std::is_arithmetic_v<UserType_> && std::is_arithmetic_v<StlType_>;
    static StlType to_stl(const UserType& u)
    {
        static_assert(IsArithmetic);
        return u;
    }
    static UserType from_stl(const StlType& s)
    {
        static_assert(IsArithmetic);
        return s;
    }
};

template <typename UserTypeA, typename UserTypeB, typename StlTypeA, typename StlTypeB>
struct DefaultSingleConverter<std::pair<UserTypeA, UserTypeB>, std::pair<StlTypeA, StlTypeB>>
{
    using UserType = std::pair<UserTypeA, UserTypeB>;
    using StlType = std::pair<StlTypeA, StlTypeB>;
    static StlType to_stl(const UserType& u)
    {
        return StlType{
            DefaultSingleConverter<UserTypeA, StlTypeA>::to_stl(u.first),
            DefaultSingleConverter<UserTypeB, StlTypeB>::to_stl(u.second)};
    }
    static UserType from_stl(const StlType& s)
    {
        return UserType{
            DefaultSingleConverter<UserTypeA, StlTypeA>::from_stl(s.first),
            DefaultSingleConverter<UserTypeB, StlTypeB>::from_stl(s.second)};
    }
};

template <typename... UserTypes, typename... StlTypes>
#if defined(__cpp_concepts)
    requires(sizeof...(UserTypes) == sizeof...(StlTypes))
#else
template <std::enable_if<sizeof...(UserTypes) == sizeof...(StlTypes), bool> = false>
#endif
struct DefaultSingleConverter<std::tuple<UserTypes...>, std::tuple<StlTypes...>>
{
    using UserType = std::tuple<UserTypes...>;
    using StlType = std::tuple<StlTypes...>;
    constexpr static size_t Size = sizeof...(UserTypes);

    template <typename A, typename B>
    static void assign(A& a, const B& b) // assigns b to a
    {
        a = b;
    }

    template <int... N>
    static StlType to_stl(const UserType& u, std::integer_sequence<int, N...>)
    {
        StlType s;
        (assign(
             std::get<N>(s),
             DefaultSingleConverter<
                 std::tuple_element_t<N, UserType>,
                 std::tuple_element_t<N, StlType>>::to_stl(std::get<N>(u))),
         ...);
        return s;
    }
    static StlType to_stl(const UserType& u)
    {
        return to_stl(u, std::make_integer_sequence<int, Size>{});
    }

    template <int... N>
    static UserType from_stl(const StlType& s, std::integer_sequence<int, N...>)
    {
        UserType u;
        (assign(
             std::get<N>(u),
             DefaultSingleConverter<
                 std::tuple_element_t<N, UserType>,
                 std::tuple_element_t<N, StlType>>::from_stl(std::get<N>(s))),
         ...);
        return u;
    }
    static UserType from_stl(const StlType& u)
    {
        return from_stl(u, std::make_integer_sequence<int, Size>{});
    }
};

template <typename ElementUserType, typename ElementStlType, size_t Size>
struct DefaultSingleConverter<std::array<ElementUserType, Size>, std::array<ElementStlType, Size>>
{
    using UserType = std::array<ElementUserType, Size>;
    using StlType = std::array<ElementStlType, Size>;

    using Converter = DefaultSingleConverter<ElementUserType, ElementStlType>;
    template <typename A, typename B>
    static void assign(A& a, const B& b) // assigns b to a
    {
        a = b;
    }

    template <int... N>
    static StlType to_stl(const UserType& u, std::integer_sequence<int, N...>)
    {
        StlType s;
        (assign(std::get<N>(s), Converter::to_stl(std::get<N>(u))), ...);
        return s;
    }
    static StlType to_stl(const UserType& u)
    {
        return to_stl(u, std::make_integer_sequence<int, Size>{});
    }

    template <int... N>
    static UserType from_stl(const StlType& s, std::integer_sequence<int, N...>)
    {
        UserType u;
        (assign(std::get<N>(u), Converter::from_stl(std::get<N>(s))), ...);
        return u;
    }
    static UserType from_stl(const StlType& u)
    {
        return from_stl(u, std::make_integer_sequence<int, Size>{});
    }
};

template <typename T, int R>
struct DefaultSingleConverter<Eigen::Matrix<T, R, 1>, std::array<T, size_t(R)>>
{
    using StlType = std::array<T, size_t(R)>;
    using UserType = Eigen::Matrix<T, R, 1>;
    static StlType to_stl(const UserType& u)
    {
        StlType s;
        typename UserType::MapType(s.data()) = u;
        return s;
    }
    static UserType from_stl(const StlType& s) { return typename UserType::ConstMapType(s.data()); }
};

template <
    typename UserType,
    typename StlType,
    typename SingleConverter =
        DefaultSingleConverter<std::decay_t<UserType>, std::decay_t<StlType>>,
    bool StlReadOnly = false,
    bool UserReadOnly = false>
class UserStlConverter : public UserStlConverterBase
{
    using StlContainerType = std::vector<StlType>;
    using UserContainerType = AttributeCollection<UserType>;
    using StlHeldType = std::conditional_t<StlReadOnly, const StlContainerType, StlContainerType>;
    using UserHeldType =
        std::conditional_t<UserReadOnly, const UserContainerType, UserContainerType>;


    UserStlConverter(StlHeldType& sv, UserContainerType& uv, SingleConverter)
        : stl_data(sv)
        , user_data(uv)
    {}
#if defined(__cpp_concepts)
    void to_stl()
        requires(!StlReadOnly)
    override
#else
    template <std::enable_if<!StlReadOnly, bool> = false>
    void to_stl() override
#endif
    {
        stl_data.resize(user_data.size());
        for (size_t j = 0; j < user_data.size(); ++j) {
            stl_data[j] = SingleConverter::to_stl(user_data[j]);
        }
    };
#if defined(__cpp_concepts)
    void from_stl()
        requires(!UserReadOnly)
    override
#else
    template <std::enable_if<!UserReadOnly, bool> = false>
    void from_stl() override
#endif
    {
        for (size_t j = 0; j < stl_data.size(); ++j) {
            user_data[j] = SingleConverter::from_stl(stl_data[j]);
        }
    }

private:
    StlHeldType& stl_data;
    UserHeldType& user_data;
};


//
//
// template <
//    typename... StlTypes,
//    typename... UserTypes,
//    typename ChildConverter = std::tuple<DefaultSingleConverter<StlTypes, UserTypes>...>>
// struct DefaultSingleConverter<std::tuple<StlTypes...>, std::tuple<UserTypes...>>
//{
//    using StlType = std::tuple<StlTypes...>;
//    using UserType = std::tuple<UserTypes...>;
//    using UserTieType = std::tie<UserTypes...>;
//    template <int... N, typename UType>
//    static StlType to_stl(const UType& u, std::integer_sequence<int, N...>)
//    {
//        StlType s;
//        (std::get<N>(s).operator=(std::tuple_element_t<N, ChildConverter>{}, std::get<N>(u)),
//        ...); return s;
//    }
//    static StlType to_stl(const UserType& u)
//    {
//        return to_stl(u, std::make_integer_sequence<int, sizeof...(Type)>{});
//    }
//    // add some extra functionality for tie convenience. induces implicit copies that hopefully get
//    // elided out for trivially copyable objects or whatnot
//    static StlType to_stl(const UserTieType& u)
//    {
//        return to_stl(u, std::make_integer_sequence<int, sizeof...(Type)>{});
//    }
//
//    template <int... N>
//    static UserType from_stl(const StlType& s, std::integer_sequence<int, N...>)
//    {
//        UserType u;
//        (std::get<N>(u).operator=(std::tuple_element_t<N, ChildConverter>{}, std::get<N>(s)),
//        ...); return u;
//    }
//    static UserType from_stl(const StlType& s)
//    {
//        return from_stl(s, std::make_integer_sequence<int, sizeof...(Type)>{});
//    }
//};
// template <typename T, int R>
// struct DefaultSingleConverter<std::array<T, size_t(R)>, Eigen::Matrix<T, R, 1>>
//{
//    using StlType = std::array<T, size_t(R)>;
//    using UserType = Eigen::Matrix<T, R, 1>;
//
//    static StlType to_stl(const UserType& u)
//    {
//        StlType s;
//        static_assert(R != Eigen::Dynamic);
//        UserType::MapType(s.data()) = u;
//        return s;
//    }
//
//    static UserType from_stl(const StlType& s)
//    {
//        static_assert(R != Eigen::Dynamic);
//        return UserType::MapType(s.data());
//    }
//};
//
//
// template <typename Derived, typename UserType, typename StlType>
// class AttributeCollectionStlConverter
//{
//    const Derived& derived() const { return static_cast<const Derived&>(*this); }
//    Derived& derived() { return static_cast<Derived&>(*this); }
//
//    StlType to_stl(const UserType& ut) template <typename UserType, typename StlType>
//    UserType from_stl(const StlType& st);
//};
//
//
// template <typename UserType, typename StlType>
// std::vector<StlType> to_stl_vector(const Attributecollection<UserType>& ut)
//{
//    std::vector<StlType> ret(ut.size());
//
//    for (size_t j = 0; j < ut.size(); ++j) {
//        ret[j] = to_stl(ut[j]);
//    }
//    return ret;
//}
//
// template <typename UserType, typename StlType>
// AttributeCollection<UserType> from_stl_vector(const std::vector<StlType>& st)
//{
//    AttributeCollection<UserType> ret(st.size());
//
//    for (size_t j = 0; j < st.size(); ++j) {
//        ret[j] = from_stl(st[j]);
//    }
//    return ret;
//}
//
//
// namespace detail {
// template <typename T>
// struct StlHdf5Serialization
//{
//    static HighFive::DataType datatype() { return HighFive::create_datatype<T>(); }
//};
//
// template <typename... Type>
// struct StlHdf5Serialization<std::tuple<Type...>>
//{
//    template <int... N>
//    static HighFive::CompoundType datatype(std::integer_sequence<int, N...>)
//    {
//        using MDef = HighFive::CompoundType::member_def;
//        using TType = std::tuple<Type...>;
//        return HighFive::CompoundType(std::vector<MDef>({MDef{
//            fmt::format("t{}", N),
//            StlHdf5Serialization<std::tuple_element_t<N, TType>>::datatype()}...}));
//    }
//    static HighFive::CompoundType datatype()
//    {
//        return datatype(std::make_integer_sequence<int, sizeof...(Type)>{});
//    }
//};
//
// template <typename TA, typename TB>
// struct StlHdf5Serialization<std::pair<TA, TB>>
//{
//    static HighFive::CompoundType datatype()
//    {
//        using MDef = HighFive::CompoundType::member_def;
//        using TType = std::pair<TA, TB>;
//        return HighFive::CompoundType(std::vector<MDef>(
//            {MDef{"p0", StlHdf5Serialization<TA>::datatype()},
//             MDef{"p1", StlHdf5Serialization<TB>::datatype()}}));
//    }
//};
//
// template <typename Type, size_t M>
// struct StlHdf5Serialization<std::array<Type, M>>
//{
//    template <int... N>
//    static HighFive::DataType datatype(std::integer_sequence<int, N...>)
//    {
//        using MDef = HighFive::CompoundType::member_def;
//        return HighFive::CompoundType(std::vector<MDef>(
//            {MDef{fmt::format("a{}", N), StlHdf5Serialization<Type>::datatype()}...}));
//    }
//    static HighFive::DataType datatype() { return datatype(std::make_integer_sequence<int, M>{});
//    }
//};
} // namespace detail
} // namespace wmtk::serialization

