#pragma once
#include <fmt/format.h>
#include <cinttypes>
#include <string>
#include <tuple>

namespace wmtk::autogen {
class Dart;
class DartWrap;

template <typename IndexType, typename OrientType>
class _Dart : public std::tuple<IndexType, OrientType>
{
public:
    using tuple_type = std::tuple<IndexType, OrientType>;
    using tuple_type::tuple_type;

    IndexType& global_id() { return std::get<0>(*this); }
    IndexType global_id() const { return std::get<0>(*this); }

    OrientType& local_orientation() { return std::get<1>(*this); }
    OrientType local_orientation() const { return std::get<1>(*this); }

    bool is_null() const { return global_id() == -1; }

    operator std::string() const
    {
        return fmt::format("Dart[{}:{}]", global_id(), local_orientation());
    }

    const tuple_type& as_tuple() const { return static_cast<const tuple_type&>(*this); }

    template <typename A, typename B>
    _Dart& operator=(const _Dart<A, B>& o)
    {
        tuple_type::operator=(o.as_tuple());
        return *this;
    }
};

template <int64_t Dim>
class SimplexAdjacency;

class Dart : public _Dart<int64_t, int8_t>
{
public:
    template <int64_t Dim>
    friend class SimplexAdjacency;
    using _DartType = _Dart<int64_t, int8_t>;
    using _DartType::_DartType;
    using _DartType::global_id;
    using _DartType::is_null;
    using _DartType::local_orientation;
    using _DartType::operator=;
    Dart()
        : _Dart(-1, -1)
    {}
    template <typename A, typename B>
    Dart(const _Dart<A, B>& o)
        : _DartType(o.global_id(), o.local_orientation())
    {}
};

class DartWrap : public _Dart<int64_t&, int8_t&>
{
public:
    using _DartType = _Dart<int64_t&, int8_t&>;
    using _DartType::_DartType;
    using _DartType::global_id;
    using _DartType::is_null;
    using _DartType::local_orientation;
    using _DartType::operator=;
};
} // namespace wmtk::autogen
