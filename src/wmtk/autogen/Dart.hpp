#pragma once
#include <cinttypes>
#include <string>
#include <tuple>

namespace wmtk::autogen {
class Dart : public std::tuple<int64_t, int8_t>
{
public:
    using tuple_type = std::tuple<int64_t, int8_t>;
    using tuple_type::tuple_type;
    Dart()
        : Dart(-1, -1)
    {}
    int64_t& global_id() { return std::get<0>(*this); }
    int64_t global_id() const { return std::get<0>(*this); }

    int8_t& local_orientation() { return std::get<1>(*this); }
    int8_t local_orientation() const { return std::get<1>(*this); }

    bool is_null() const { return global_id() == -1; }

    operator std::string() const;

    const tuple_type& as_tuple() const { return static_cast<const tuple_type&>(*this); }
};
} // namespace wmtk::autogen
auto operator<<(std::ostream& out, const wmtk::autogen::Dart& vec) -> std::ostream&;
