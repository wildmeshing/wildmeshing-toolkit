#pragma once
#include <array>
#include <cstdint>

namespace wmtk {
class Mesh;
class Tuple;

} // namespace wmtk
namespace wmtk::tests::tools {

// class to friend with Mesh class. Add your test tool funcs as static members here and leave the,
// implementations lie in individual functions's cpp files
class TestTools
{
public:
    static std::array<int64_t, 4> global_ids(const Mesh& m, const Tuple& a);

    static int64_t global_id(const Mesh& m, const Tuple& a, const PrimitiveType& pt);
};
} // namespace wmtk::tests::tools
