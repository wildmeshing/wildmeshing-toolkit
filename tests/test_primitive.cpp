#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/utils/primitive_range.hpp>

using namespace wmtk;
TEST_CASE("primitive_range", "[primitive]")
{
    for (PrimitiveType pt :
         {PrimitiveType::Vertex,
          PrimitiveType::Edge,
          PrimitiveType::Triangle,
          PrimitiveType::Tetrahedron}) {
        {
            auto a = wmtk::utils::primitive_range(pt, PrimitiveType::Tetrahedron);
            auto b = wmtk::utils::primitive_above(pt);
            CHECK(a == b);
        }
        {
            auto a = wmtk::utils::primitive_range(PrimitiveType::Vertex, pt);
            auto b = wmtk::utils::primitive_below(pt);
            std::reverse(b.begin(), b.end());
            CHECK(a == b);
        }
    }
    // 1,1
    // 1,2
    // 2,2
    {
        auto a = wmtk::utils::primitive_range(PrimitiveType::Edge, PrimitiveType::Edge);
        std::vector<PrimitiveType> b{PrimitiveType::Edge};
        CHECK(a == b);
    }
    {
        auto a = wmtk::utils::primitive_range(PrimitiveType::Triangle, PrimitiveType::Triangle);
        std::vector<PrimitiveType> b{PrimitiveType::Triangle};
        CHECK(a == b);
    }
    {
        auto a = wmtk::utils::primitive_range(PrimitiveType::Edge, PrimitiveType::Triangle);
        std::vector<PrimitiveType> b{PrimitiveType::Edge, PrimitiveType::Triangle};
        CHECK(a == b);
    }
}
TEST_CASE("primitive_above", "[primitive]")
{
    {
        auto a = wmtk::utils::primitive_above(PrimitiveType::Tetrahedron);
        std::vector<PrimitiveType> b{PrimitiveType::Tetrahedron};
        CHECK(a == b);
    }
    {
        auto a = wmtk::utils::primitive_above(PrimitiveType::Triangle);
        std::vector<PrimitiveType> b{PrimitiveType::Triangle, PrimitiveType::Tetrahedron};
        CHECK(a == b);
    }
    {
        auto a = wmtk::utils::primitive_above(PrimitiveType::Edge);
        std::vector<PrimitiveType> b{
            PrimitiveType::Edge,
            PrimitiveType::Triangle,
            PrimitiveType::Tetrahedron,
        };
        CHECK(a == b);
    }
    {
        auto a = wmtk::utils::primitive_above(PrimitiveType::Vertex);
        std::vector<PrimitiveType> b{
            PrimitiveType::Vertex,
            PrimitiveType::Edge,
            PrimitiveType::Triangle,
            PrimitiveType::Tetrahedron};
        CHECK(a == b);
    }
}
TEST_CASE("primitive_below", "[primitive]")
{
    {
        auto a = wmtk::utils::primitive_below(PrimitiveType::Tetrahedron);
        std::vector<PrimitiveType> b{
            PrimitiveType::Tetrahedron,
            PrimitiveType::Triangle,
            PrimitiveType::Edge,
            PrimitiveType::Vertex,
        };
        CHECK(a == b);
    }
    {
        auto a = wmtk::utils::primitive_below(PrimitiveType::Triangle);
        std::vector<PrimitiveType> b{
            PrimitiveType::Triangle,
            PrimitiveType::Edge,
            PrimitiveType::Vertex,
        };
        CHECK(a == b);
    }
    {
        auto a = wmtk::utils::primitive_below(PrimitiveType::Edge);
        std::vector<PrimitiveType> b{
            PrimitiveType::Edge,
            PrimitiveType::Vertex,
        };
        CHECK(a == b);
    }
    {
        auto a = wmtk::utils::primitive_below(PrimitiveType::Vertex);
        std::vector<PrimitiveType> b{PrimitiveType::Vertex};
        CHECK(a == b);
    }
}
