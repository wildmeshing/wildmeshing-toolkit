#include <catch2/catch_test_macros.hpp>

#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/cast_attribute.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "tools/DEBUG_EdgeMesh.hpp"
#include "tools/DEBUG_PointMesh.hpp"
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/EdgeMesh_examples.hpp"
#include "tools/TriMesh_examples.hpp"

#include <wmtk/invariants/EnvelopeInvariant.hpp>

using namespace wmtk;
using namespace tests;
using namespace invariants;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;

void positions_as_rational(Mesh& mesh)
{
    auto pt_double_attribute = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    wmtk::utils::cast_attribute<wmtk::Rational>(pt_double_attribute, mesh, "vertices");
    mesh.delete_attribute(pt_double_attribute);
}

std::shared_ptr<DEBUG_EdgeMesh> construct_edge_45(DEBUG_TriMesh& m)
{
    auto env_pos_handle = m.get_attribute_handle<double>("vertices", PV);
    auto acc = m.create_const_accessor<double>(env_pos_handle);

    std::shared_ptr<DEBUG_EdgeMesh> em_ptr = std::make_shared<DEBUG_EdgeMesh>(single_line());
    Eigen::MatrixXd V;
    V.resize(2, 3);
    V.row(0) = acc.const_vector_attribute(m.vertex_tuple_from_id(4));
    V.row(1) = acc.const_vector_attribute(m.vertex_tuple_from_id(5));
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, *em_ptr);

    return em_ptr;
}

std::shared_ptr<DEBUG_PointMesh> construct_point_4(DEBUG_TriMesh& m)
{
    auto env_pos_handle = m.get_attribute_handle<double>("vertices", PV);
    auto acc = m.create_const_accessor<double>(env_pos_handle);

    std::shared_ptr<DEBUG_PointMesh> pm_ptr = std::make_shared<DEBUG_PointMesh>(1);
    Eigen::MatrixXd V;
    V.resize(1, 3);
    V.row(0) = acc.const_vector_attribute(m.vertex_tuple_from_id(4));
    mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, *pm_ptr);

    return pm_ptr;
}

TEST_CASE("envelope_invariant_envelope", "[invariants][envelope]")
{
    std::shared_ptr<DEBUG_TriMesh> mesh_in =
        std::make_shared<DEBUG_TriMesh>(edge_region_with_position());
    DEBUG_TriMesh& m = *mesh_in;


    SECTION("double_triangle")
    {
        using T = double;

        auto env_pos_handle = m.get_attribute_handle<T>("vertices", PV);
        EnvelopeInvariant env_inv(env_pos_handle, 1e-2, env_pos_handle);

        const Tuple v4 = m.vertex_tuple_from_id(4);
        const simplex::Simplex s4(PV, v4);

        const auto tops = simplex::top_dimension_cofaces_tuples(m, s4);
        CHECK(env_inv.after({}, tops));

        auto acc = m.create_accessor<T>(env_pos_handle);
        acc.vector_attribute(s4)[2] = 1;
        CHECK_FALSE(env_inv.after({}, tops));
    }
    SECTION("rational_triangle")
    {
        using T = Rational;
        positions_as_rational(m);

        auto env_pos_handle = m.get_attribute_handle<T>("vertices", PV);
        EnvelopeInvariant env_inv(env_pos_handle, 1e-2, env_pos_handle);

        const Tuple v4 = m.vertex_tuple_from_id(4);
        const simplex::Simplex s4(PV, v4);

        const auto tops = simplex::top_dimension_cofaces_tuples(m, s4);
        CHECK(env_inv.after({}, tops));

        auto acc = m.create_accessor<T>(env_pos_handle);
        acc.vector_attribute(s4)[2] = 1;
        CHECK_FALSE(env_inv.after({}, tops));
    }
    SECTION("double_edge")
    {
        std::shared_ptr<DEBUG_EdgeMesh> em_ptr = construct_edge_45(m);
        DEBUG_EdgeMesh& em = *em_ptr;

        using T = double;

        auto env_pos_handle = m.get_attribute_handle<double>("vertices", PV);
        auto query_pos_handle = em.get_attribute_handle<T>("vertices", PV);
        EnvelopeInvariant env_inv(env_pos_handle, 1e-2, query_pos_handle);

        const Tuple v0 = em.tuple_from_edge_id(0);
        const simplex::Simplex s0(PV, v0);


        const auto tops = simplex::top_dimension_cofaces_tuples(em, s0);
        CHECK(env_inv.after({}, tops));

        auto acc = em.create_accessor<T>(query_pos_handle);
        acc.vector_attribute(s0)[2] = 1;
        CHECK_FALSE(env_inv.after({}, tops));
    }
    SECTION("rational_edge")
    {
        std::shared_ptr<DEBUG_EdgeMesh> em_ptr = construct_edge_45(m);
        DEBUG_EdgeMesh& em = *em_ptr;

        using T = Rational;
        positions_as_rational(em);

        auto env_pos_handle = m.get_attribute_handle<double>("vertices", PV);
        auto query_pos_handle = em.get_attribute_handle<T>("vertices", PV);
        EnvelopeInvariant env_inv(env_pos_handle, 1e-2, query_pos_handle);

        const Tuple v0 = em.tuple_from_edge_id(0);
        const simplex::Simplex s0(PV, v0);

        const auto tops = simplex::top_dimension_cofaces_tuples(em, s0);
        CHECK(env_inv.after({}, tops));

        auto acc = em.create_accessor<T>(query_pos_handle);
        acc.vector_attribute(s0)[2] = 1;
        CHECK_FALSE(env_inv.after({}, tops));
    }
    SECTION("double_point")
    {
        std::shared_ptr<DEBUG_PointMesh> pm_ptr = construct_point_4(m);
        DEBUG_PointMesh& pm = *pm_ptr;

        using T = double;

        auto env_pos_handle = m.get_attribute_handle<double>("vertices", PV);
        auto query_pos_handle = pm.get_attribute_handle<T>("vertices", PV);
        EnvelopeInvariant env_inv(env_pos_handle, 1e-2, query_pos_handle);

        const Tuple v0(-1, -1, -1, 0);
        const simplex::Simplex s0(PV, v0);

        const auto tops = simplex::top_dimension_cofaces_tuples(pm, s0);
        CHECK(env_inv.after({}, tops));

        auto acc = pm.create_accessor<T>(query_pos_handle);
        acc.vector_attribute(s0)[2] = 1;
        CHECK_FALSE(env_inv.after({}, tops));
    }
    SECTION("rational_point")
    {
        std::shared_ptr<DEBUG_PointMesh> pm_ptr = construct_point_4(m);
        DEBUG_PointMesh& pm = *pm_ptr;

        using T = Rational;
        positions_as_rational(pm);

        auto env_pos_handle = m.get_attribute_handle<double>("vertices", PV);
        auto query_pos_handle = pm.get_attribute_handle<T>("vertices", PV);
        EnvelopeInvariant env_inv(env_pos_handle, 1e-2, query_pos_handle);

        const Tuple v0(-1, -1, -1, 0);
        const simplex::Simplex s0(PV, v0);

        const auto tops = simplex::top_dimension_cofaces_tuples(pm, s0);
        CHECK(env_inv.after({}, tops));

        auto acc = pm.create_accessor<T>(query_pos_handle);
        acc.vector_attribute(s0)[2] = 1;
        CHECK_FALSE(env_inv.after({}, tops));
    }
}

TEST_CASE("envelope_invariant_bvh", "[invariants][envelope]")
{
    std::shared_ptr<DEBUG_EdgeMesh> em_ptr = std::make_shared<DEBUG_EdgeMesh>(single_line());
    DEBUG_EdgeMesh& em = *em_ptr;
    {
        Eigen::MatrixXd V;
        V.resize(2, 2);
        V.row(0) << 0, 0;
        V.row(1) << 1, 0;
        mesh_utils::set_matrix_attribute(V, "vertices", PV, em);
    }

    SECTION("double_edge")
    {
        using T = double;

        auto env_pos_handle = em.get_attribute_handle<T>("vertices", PV);
        EnvelopeInvariant env_inv(env_pos_handle, 1e-2, env_pos_handle);

        const Tuple v0 = em.tuple_from_edge_id(0);
        const simplex::Simplex s0(PV, v0);

        const auto tops = simplex::top_dimension_cofaces_tuples(em, s0);
        CHECK(env_inv.after({}, tops));

        auto acc = em.create_accessor<T>(env_pos_handle);
        acc.vector_attribute(s0)[1] = 1;
        CHECK_FALSE(env_inv.after({}, tops));
    }
    SECTION("rational_edge")
    {
        using T = Rational;
        positions_as_rational(em);

        auto env_pos_handle = em.get_attribute_handle<T>("vertices", PV);
        EnvelopeInvariant env_inv(env_pos_handle, 1e-2, env_pos_handle);

        const Tuple v0 = em.tuple_from_edge_id(0);
        const simplex::Simplex s0(PV, v0);

        const auto tops = simplex::top_dimension_cofaces_tuples(em, s0);
        CHECK(env_inv.after({}, tops));

        auto acc = em.create_accessor<T>(env_pos_handle);
        acc.vector_attribute(s0)[1] = 1;
        CHECK_FALSE(env_inv.after({}, tops));
    }
    SECTION("double_point")
    {
        std::shared_ptr<DEBUG_PointMesh> pm_ptr = std::make_shared<DEBUG_PointMesh>(1);
        DEBUG_PointMesh& pm = *pm_ptr;

        using T = double;

        auto env_pos_handle = em.get_attribute_handle<double>("vertices", PV);
        auto query_pos_handle = pm.register_attribute<T>("vertices", PV, 2);
        EnvelopeInvariant env_inv(env_pos_handle, 1e-2, query_pos_handle);

        const Tuple v0(-1, -1, -1, 0);
        const simplex::Simplex s0(PV, v0);

        const auto tops = simplex::top_dimension_cofaces_tuples(pm, s0);
        CHECK(env_inv.after({}, tops));

        auto acc = pm.create_accessor<T>(query_pos_handle);
        acc.vector_attribute(s0)[1] = 1;
        CHECK_FALSE(env_inv.after({}, tops));
    }
    SECTION("rational_point")
    {
        std::shared_ptr<DEBUG_PointMesh> pm_ptr = std::make_shared<DEBUG_PointMesh>(1);
        DEBUG_PointMesh& pm = *pm_ptr;

        using T = Rational;

        auto env_pos_handle = em.get_attribute_handle<double>("vertices", PV);
        auto query_pos_handle = pm.register_attribute<T>("vertices", PV, 2);
        EnvelopeInvariant env_inv(env_pos_handle, 1e-2, query_pos_handle);

        const Tuple v0(-1, -1, -1, 0);
        const simplex::Simplex s0(PV, v0);

        const auto tops = simplex::top_dimension_cofaces_tuples(pm, s0);
        CHECK(env_inv.after({}, tops));

        auto acc = pm.create_accessor<T>(query_pos_handle);
        acc.vector_attribute(s0)[1] = 1;
        CHECK_FALSE(env_inv.after({}, tops));
    }
}