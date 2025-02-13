#include <catch2/catch_test_macros.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/invariants/internal/ConstantInvariant.hpp>
#include <wmtk/multimesh/BoundaryChecker.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/utils/primitive_range.hpp>
#include "../tools/DEBUG_EdgeMesh.hpp"
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/EdgeMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::simplex;

using TM = TriMesh;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;


TEST_CASE("multimesh_boundary", "[multimesh]")
{
    // creates N triangles surrounding a single interior vertex 0
    int number = 20;
    auto dptr = disk_to_individual_multimesh(number);
    REQUIRE(dptr->get_all_child_meshes().size() == 1);
    auto& c = dptr->get_multi_mesh_child_mesh({0});

    {
        auto tag_handle = c.register_attribute<int64_t>("is_child", wmtk::PrimitiveType::Edge, 1);
        auto tag_accessor = c.create_accessor(tag_handle.as<int64_t>());
        for (const auto& e : c.get_all(PE)) {
            tag_accessor.scalar_attribute(e) = c.is_boundary(PrimitiveType::Edge, e) ? 1 : 0;
        }
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(tag_handle, int64_t(1));
    }
    auto& edge_mesh = dptr->get_multi_mesh_child_mesh({0, 0});


    { // check that boundarychecker does the normal boundary fine
        multimesh::BoundaryChecker bc(*dptr);
        for (PrimitiveType pt : wmtk::utils::primitive_below(PrimitiveType::Edge)) {
            auto simplices = dptr->get_all(pt);
            // c and dptr have the same triangles so no mapping is required

            for (const auto& s : simplices) {
                CHECK(dptr->is_boundary(pt, s) == bc.is_boundary(*dptr, pt, s));
            }
        }
    }
    {
        multimesh::BoundaryChecker bc(*dptr, c);
        for (PrimitiveType pt : wmtk::utils::primitive_below(PrimitiveType::Edge)) {
            auto simplices = dptr->get_all(pt);
            // c and dptr have the same triangles so no mapping is required
            // also, dptr is_boundary implies c is_boundary
            for (const auto& s : simplices) {
                CHECK(c.is_boundary(pt, s) == bc.is_boundary(*dptr, pt, s));
            }
        }
    }
    {
        multimesh::BoundaryChecker bc(*dptr, edge_mesh);
        for (PrimitiveType pt : wmtk::utils::primitive_below(PrimitiveType::Edge)) {
            auto simplices = dptr->get_all(pt);
            // the edge mesh was constructed by lookign at the boundary of c
            // c and dptr have the same triangles so no mapping is required
            // also, dptr is_boundary implies c is_boundary
            for (const auto& s : simplices) {
                CHECK(c.is_boundary(pt, s) == bc.is_boundary(*dptr, pt, s));
            }
        }
    }
}
