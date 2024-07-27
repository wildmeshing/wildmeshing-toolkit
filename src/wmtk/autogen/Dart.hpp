#pragma once
#include <cinttypes>
#include <tuple>

namespace wmtk::autogen {
class Dart : public std::tuple<int64_t, int8_t>
{
public:
    using std::tuple<int64_t, int8_t>::tuple;
    Dart()
        : Dart(-1, -1)
    {}
    int64_t& global_id() { return std::get<0>(*this); }
    int64_t global_id() const { return std::get<0>(*this); }

    int8_t& local_orientation() { return std::get<1>(*this); }
    int8_t local_orientation() const { return std::get<1>(*this); }
};
} // namespace wmtk::autogen
