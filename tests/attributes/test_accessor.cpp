#include <numeric>

#include <catch2/catch_test_macros.hpp>
#include <wmtk/attribute/Attribute.hpp>
#include <wmtk/utils/Logger.hpp>
#include "../tools/DEBUG_PointMesh.hpp"
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"


using namespace wmtk::tests;
namespace {


template <typename VectorAcc>
void populate(DEBUG_PointMesh& m, VectorAcc& va, bool for_zeros = false)
{
    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    size_t dimension = va.dimension();
    Eigen::Matrix<typename VectorAcc::Scalar, Eigen::Dynamic, 1> x;
    for (const wmtk::Tuple& tup : vertices) {
        int64_t id = m.id(tup);
        auto v = va.vector_attribute(tup);
        if (for_zeros) {
            v.setZero();
        } else {
            std::iota(v.begin(), v.end(), (typename VectorAcc::Scalar)(dimension * id));
        }
    }
}
template <typename VectorAcc>
void check(DEBUG_PointMesh& m, VectorAcc& va, bool for_zeros = false)
{
    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    size_t dimension = va.dimension();
    Eigen::Matrix<typename VectorAcc::Scalar, Eigen::Dynamic, 1> x;
    bool is_scalar = va.dimension() == 1;
    x.resize(va.dimension());
    for (const wmtk::Tuple& tup : vertices) {
        int64_t id = m.id(tup);
        if (for_zeros) {
            CHECK((va.const_vector_attribute(tup).array() == 0).all());
            if (is_scalar) {
                CHECK(va.const_scalar_attribute(tup) == 0);
            }
        } else {
            auto v = va.vector_attribute(tup);
            std::iota(x.begin(), x.end(), (typename VectorAcc::Scalar)(dimension * id));
            CHECK(v == x);
            if (is_scalar) {
                CHECK(va.const_scalar_attribute(tup) == id);
            }
        }
    }
}
} // namespace

TEST_CASE("test_accessor_basic", "[accessor]")
{
    int64_t size = 20;
    DEBUG_PointMesh m(size);
    REQUIRE(size == m.capacity(wmtk::PrimitiveType::Vertex));
    auto char_handle = m.register_attribute_typed<char>("char", wmtk::PrimitiveType::Vertex, 1);
    auto int64_t_handle =
        m.register_attribute_typed<int64_t>("int64_t", wmtk::PrimitiveType::Vertex, 1);
    auto double_handle =
        m.register_attribute_typed<double>("double", wmtk::PrimitiveType::Vertex, 3);

    auto char_def1_handle =
        m.register_attribute_typed<char>("char1", wmtk::PrimitiveType::Vertex, 1, false, 1);
    auto int64_t_def1_handle =
        m.register_attribute_typed<int64_t>("int64_t1", wmtk::PrimitiveType::Vertex, 1, false, 1);
    auto double_def1_handle =
        m.register_attribute_typed<double>("double1", wmtk::PrimitiveType::Vertex, 3, false, 1);

    REQUIRE(m.get_attribute_dimension(char_handle) == 1);
    REQUIRE(m.get_attribute_dimension(int64_t_handle) == 1);
    REQUIRE(m.get_attribute_dimension(double_handle) == 3);

    REQUIRE(m.get_attribute_dimension(char_def1_handle) == 1);
    REQUIRE(m.get_attribute_dimension(int64_t_def1_handle) == 1);
    REQUIRE(m.get_attribute_dimension(double_def1_handle) == 3);

    auto char_acc = m.create_accessor(char_handle);
    auto int64_t_acc = m.create_accessor(int64_t_handle);
    auto double_acc = m.create_accessor(double_handle);

    auto char_def1_acc = m.create_accessor(char_def1_handle);
    auto int64_t_def1_acc = m.create_accessor(int64_t_def1_handle);
    auto double_def1_acc = m.create_accessor(double_def1_handle);

    auto char_bacc = m.create_base_accessor(char_handle);
    auto int64_t_bacc = m.create_base_accessor(int64_t_handle);
    auto double_bacc = m.create_base_accessor(double_handle);

    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);

    // check characteristics are all right
    REQUIRE(char_acc.reserved_size() == size);
    REQUIRE(int64_t_acc.reserved_size() == size);
    REQUIRE(double_acc.reserved_size() == size);
    REQUIRE(char_acc.dimension() == 1);
    REQUIRE(int64_t_acc.dimension() == 1);
    REQUIRE(double_acc.dimension() == 3);

    // test default initialization to 0
    for (const wmtk::Tuple& tup : vertices) {
        const wmtk::simplex::Simplex s(m, wmtk::PrimitiveType::Vertex, tup);
        CHECK(char_acc.const_scalar_attribute(tup) == 0);
        CHECK(int64_t_acc.const_scalar_attribute(tup) == 0);
        CHECK((double_acc.const_vector_attribute(tup).array() == 0).all());

        CHECK(char_acc.const_scalar_attribute(s) == 0);
        CHECK(int64_t_acc.const_scalar_attribute(s) == 0);
        CHECK((double_acc.const_vector_attribute(s).array() == 0).all());

        // checking that default initialization of 1 worked
        CHECK(char_def1_acc.const_scalar_attribute(tup) == 1);
        CHECK(int64_t_def1_acc.const_scalar_attribute(tup) == 1);
        CHECK((double_def1_acc.const_vector_attribute(tup).array() == 1).all());

        CHECK(char_def1_acc.const_scalar_attribute(s) == 1);
        CHECK(int64_t_def1_acc.const_scalar_attribute(s) == 1);
        CHECK((double_def1_acc.const_vector_attribute(s).array() == 1).all());
    }

    // use global set to force all values
    // NOTE that the ugly static casts are in this unit test because we want
    // accessing the low level accessor data to be ugly.
    // Please keep set_attribute hidden from the public unless some ugly
    // notation like these static asts exists
    {
        std::vector<char> d(size);
        std::iota(d.begin(), d.end(), char(0));
        char_bacc.set_attribute(d);
    }
    {
        std::vector<int64_t> d(size);
        std::iota(d.begin(), d.end(), int64_t(0));
        int64_t_bacc.set_attribute(d);
    }
    {
        std::vector<double> d(3 * size);
        std::iota(d.begin(), d.end(), double(0));
        double_bacc.set_attribute(d);
    }
    for (const wmtk::Tuple& tup : vertices) {
        int64_t id = m.id(tup);
        CHECK(char_acc.const_scalar_attribute(tup) == char(id));
        CHECK(int64_t_acc.const_scalar_attribute(tup) == id);
        Eigen::Vector3d x(3 * id, 3 * id + 1, 3 * id + 2);
        CHECK((double_acc.const_vector_attribute(tup) == x));
    }

    { // check const accessors
        auto char_cacc = m.create_const_accessor(char_handle);
        auto int64_t_cacc = m.create_const_accessor(int64_t_handle);
        auto double_cacc = m.create_const_accessor(double_handle);
        for (const wmtk::Tuple& tup : vertices) {
            int64_t id = m.id(tup);
            CHECK(char_cacc.const_scalar_attribute(tup) == char(id));
            CHECK(int64_t_cacc.const_scalar_attribute(tup) == id);
            Eigen::Vector3d x(3 * id, 3 * id + 1, 3 * id + 2);
            CHECK((double_cacc.const_vector_attribute(tup) == x));
        }
    }
}

TEST_CASE("test_accessor_caching", "[accessor]")
{
    int64_t size = 20;
    DEBUG_PointMesh m(size);
    REQUIRE(size == m.capacity(wmtk::PrimitiveType::Vertex));
    auto int64_t_handle =
        m.register_attribute_typed<int64_t>("int64_t", wmtk::PrimitiveType::Vertex, 1);
    auto double_handle =
        m.register_attribute_typed<double>("double", wmtk::PrimitiveType::Vertex, 3);
    auto immediate_int64_t_acc = m.create_base_accessor(int64_t_handle);
    auto immediate_double_acc = m.create_base_accessor(double_handle);

    std::vector<int64_t*> int64_t_ptrs;
    std::vector<double*> double_ptrs;
    for (int64_t j = 0; j < m.capacity(wmtk::PrimitiveType::Vertex); ++j) {
        int64_t_ptrs.emplace_back(&immediate_int64_t_acc.vector_attribute(j)(0));
        double_ptrs.emplace_back(&immediate_double_acc.vector_attribute(j)(0));
    }

    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);

    REQUIRE(int64_t(vertices.size()) == size);

    {
        wmtk::logger().trace("Creating a scope");
        // TODO: create scope
        auto scope = m.create_scope();
        {
            // make sure base accessors are not affected
            for (int64_t j = 0; j < m.capacity(wmtk::PrimitiveType::Vertex); ++j) {
                CHECK(&immediate_int64_t_acc.vector_attribute(j)(0) == int64_t_ptrs[j]);
                CHECK(&immediate_double_acc.vector_attribute(j)(0) == double_ptrs[j]);
            }
        }
        auto int64_t_acc = m.create_accessor(int64_t_handle);
        auto double_acc = m.create_accessor(double_handle);
        {
            auto depth = int64_t_acc.stack_depth();
            REQUIRE(depth == 1);

            // make sure base accessors are not affected
            for (const wmtk::Tuple& tup : vertices) {
                int64_t id = m.id(tup);
                REQUIRE(int64_t_acc.vector_attribute(tup).size() == 1);
                REQUIRE(double_acc.vector_attribute(tup).size() == 3);
                CHECK(&int64_t_acc.vector_attribute(tup)(0) == int64_t_ptrs[id]);
                CHECK(&double_acc.vector_attribute(tup)(0) == double_ptrs[id]);
            }
        }

        // check characteristics are all right
        REQUIRE(int64_t_acc.reserved_size() == size);
        REQUIRE(double_acc.reserved_size() == size);
        REQUIRE(int64_t_acc.dimension() == 1);
        REQUIRE(double_acc.dimension() == 3);

        // use global set to force all values

        populate(m, int64_t_acc, false);
        populate(m, double_acc, false);
        check(m, int64_t_acc, false);
        check(m, double_acc, false);


        for (const wmtk::Tuple& tup : vertices) {
            auto check_id = [&](const auto& va, int id) {
                using T = typename std::decay_t<decltype(va)>::T;
                auto v = va.const_vector_attribute(id);
                auto x = v.eval();
                std::iota(x.begin(), x.end(), T(va.dimension() * id));
                CHECK(v == x);
                bool is_scalar = va.dimension() == 1;
                if (is_scalar) {
                    CHECK(va.const_scalar_attribute(id) == id);
                }
            };
            int64_t id = m.id(tup);
            check_id(immediate_int64_t_acc, id);
            check_id(immediate_double_acc, id);
        }
    }
    // test that the accessors above unbuffered when they finished scope
    for (const wmtk::Tuple& tup : vertices) {
        int64_t id = m.id(tup);
        CHECK(immediate_int64_t_acc.const_scalar_attribute(id) == id);
        Eigen::Vector3d x(3 * id, 3 * id + 1, 3 * id + 2);
        CHECK((immediate_double_acc.const_vector_attribute(id) == x));
    }
}

TEST_CASE("test_accessor_caching_scope_fails", "[accessor][caching]")
{
    int64_t size = 20;
    DEBUG_PointMesh m(size);
    REQUIRE(size == m.capacity(wmtk::PrimitiveType::Vertex));
    auto int64_t_handle =
        m.register_attribute_typed<int64_t>("int64_t", wmtk::PrimitiveType::Vertex, 1);
    auto double_handle =
        m.register_attribute_typed<double>("double", wmtk::PrimitiveType::Vertex, 3);
    auto int64_t_acc = m.create_accessor(int64_t_handle);
    auto double_acc = m.create_accessor(double_handle);
    {
        wmtk::logger().info("Creating a scope");
        // TODO: create scope
        auto scope = m.create_scope();

        populate(m, int64_t_acc, false);
        populate(m, double_acc, false);
        check(m, int64_t_acc, false);
        check(m, double_acc, false);

        spdlog::info("Walking out of scope");
        scope.mark_failed();
    }
    check(m, int64_t_acc, true);
    check(m, double_acc, true);
}
TEST_CASE("test_accessor_caching_scope_success_fails", "[accessor]")
{
    int64_t size = 20;
    DEBUG_PointMesh m(size);
    REQUIRE(size == m.capacity(wmtk::PrimitiveType::Vertex));
    auto int64_t_handle =
        m.register_attribute_typed<int64_t>("int64_t", wmtk::PrimitiveType::Vertex, 1);
    auto double_handle =
        m.register_attribute_typed<double>("double", wmtk::PrimitiveType::Vertex, 3);
    auto int64_t_acc = m.create_accessor(int64_t_handle);
    auto double_acc = m.create_accessor(double_handle);
    {
        wmtk::logger().info("Creating a scope");
        // TODO: create scope

        auto scope = m.create_scope();
        populate(m, int64_t_acc, false);
        populate(m, double_acc, false);
        check(m, int64_t_acc, false);
        check(m, double_acc, false);
        {
            auto scope2 = m.create_scope();
            populate(m, int64_t_acc, true);
            populate(m, double_acc, true);
            check(m, int64_t_acc, true);
            check(m, double_acc, true);

            scope2.mark_failed();
        }
    }
    check(m, int64_t_acc, false);
    check(m, double_acc, false);
}
TEST_CASE("test_accessor_caching_scope_fails_success", "[accessor][caching]")
{
    int64_t size = 20;
    DEBUG_PointMesh m(size);
    REQUIRE(size == m.capacity(wmtk::PrimitiveType::Vertex));
    auto int64_t_handle =
        m.register_attribute_typed<int64_t>("int64_t", wmtk::PrimitiveType::Vertex, 1);
    auto double_handle =
        m.register_attribute_typed<double>("double", wmtk::PrimitiveType::Vertex, 3);
    auto int64_t_acc = m.create_accessor(int64_t_handle);
    auto double_acc = m.create_accessor(double_handle);
    populate(m, int64_t_acc, true);
    populate(m, double_acc, true);
    check(m, int64_t_acc, true);
    check(m, double_acc, true);
    {
        wmtk::logger().info("Creating a scope");
        // TODO: create scope
        auto scope = m.create_scope();

        populate(m, int64_t_acc, false);
        populate(m, double_acc, false);
        check(m, int64_t_acc, false);
        check(m, double_acc, false);
        {
            auto scope2 = m.create_scope();
            populate(m, int64_t_acc, true);
            populate(m, double_acc, true);
            check(m, int64_t_acc, true);
            check(m, double_acc, true);
        }
        scope.mark_failed();
    }
    check(m, int64_t_acc, true);
    check(m, double_acc, true);
}

TEST_CASE("accessor_parent_scope_access", "[accessor]")
{
    using namespace wmtk;

    int64_t size = 3;
    DEBUG_PointMesh m(size);
    REQUIRE(size == m.capacity(wmtk::PrimitiveType::Vertex));
    auto int64_t_handle =
        m.register_attribute_typed<int64_t>("int64_t", wmtk::PrimitiveType::Vertex, 1, 0);
    auto int64_t_acc = m.create_accessor(int64_t_handle);

    {
        auto scope = m.create_scope();

        // change value
        for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
            int64_t_acc.scalar_attribute(t) = 1;
        }

        spdlog::info("Should be walking into a scope now");
        m.parent_scope([&]() {
            for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
                CHECK(int64_t_acc.const_scalar_attribute(t) == 0);
            }
        });
        spdlog::info("Should be exiting from a scope now");

        // return a value from the parent scope
        {
            int64_t parent_value = m.parent_scope([&]() -> int64_t {
                for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
                    return int64_t_acc.const_scalar_attribute(t);
                }
                return -1;
            });
            CHECK(parent_value == 0);
        }

        // nested scopes
        {
            auto inner_scope = m.create_scope();

            // change value
            for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
                int64_t_acc.scalar_attribute(t) = 2;
            }
            // check values
            for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
                CHECK(int64_t_acc.scalar_attribute(t) == 2);
            }

            m.parent_scope([&]() {
                for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
                    CHECK(int64_t_acc.const_scalar_attribute(t) == 1);
                }
                // parent of parent
                m.parent_scope([&]() {
                    for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
                        CHECK(int64_t_acc.const_scalar_attribute(t) == 0);
                    }
                });
            });

            // check values
            for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
                CHECK(int64_t_acc.scalar_attribute(t) == 2);
            }

            inner_scope.mark_failed();
        }

        // check values
        for (const Tuple& t : m.get_all(PrimitiveType::Vertex)) {
            CHECK(int64_t_acc.scalar_attribute(t) == 1);
        }
    }
}

TEST_CASE("attribute_clear", "[attributes]")
{
    wmtk::TriMesh mold = single_equilateral_triangle(); // 0xa <- 0xa
    DEBUG_TriMesh& m = static_cast<DEBUG_TriMesh&>(mold);

    CHECK(m.custom_attributes().size() == 1);

    m.clear_attributes();
    CHECK(m.custom_attributes().size() == 0);

    {
        auto a1 = m.register_attribute_typed<char>("a1", wmtk::PrimitiveType::Vertex, 1);
        CHECK(m.custom_attributes().size() == 1);
        auto a2 = m.register_attribute_typed<char>("a2", wmtk::PrimitiveType::Vertex, 1);
        CHECK(m.custom_attributes().size() == 2);
        m.clear_attributes({a1});
        CHECK(m.custom_attributes().size() == 1);
    }

    {
        CHECK_THROWS(m.register_attribute_typed<char>("a1", wmtk::PrimitiveType::Vertex, 1));
        auto a1 = m.get_attribute_handle_typed<char>("a1", wmtk::PrimitiveType::Vertex);
        auto a2 = m.register_attribute_typed<char>("a2", wmtk::PrimitiveType::Vertex, 1);
        CHECK(m.custom_attributes().size() == 2);
        m.clear_attributes({a1, a2});
        CHECK(m.custom_attributes().size() == 2);
    }
    m.clear_attributes();
    CHECK(m.custom_attributes().size() == 0);
}

TEST_CASE("custom_attributes_vector", "[attributes]")
{
    DEBUG_TriMesh m = single_equilateral_triangle();

    CHECK(m.custom_attributes().size() == 1);
    CHECK(m.get_attribute_name(m.custom_attributes()[0]) == "vertices");

    auto handle =
        m.register_attribute_typed<double>("vertices", wmtk::PrimitiveType::Vertex, 3, true);

    CHECK(m.custom_attributes().size() == 1);
}
