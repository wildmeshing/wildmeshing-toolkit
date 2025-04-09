#include <numeric>

#include <catch2/catch_test_macros.hpp>
#include <wmtk/attribute/Attribute.hpp>
#include <wmtk/utils/Logger.hpp>
#include "../tools/DEBUG_PointMesh.hpp"
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"


using namespace wmtk;
using namespace tests;
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
    auto vertices_id = m.get_all_id_simplex(wmtk::PrimitiveType::Vertex);
    size_t dimension = va.dimension();
    Eigen::Matrix<typename VectorAcc::Scalar, Eigen::Dynamic, 1> x;
    bool is_scalar = va.dimension() == 1;
    x.resize(va.dimension());
    for (size_t j = 0; j < vertices.size(); ++j) {
        const wmtk::Tuple& tup = vertices[j];
        const wmtk::simplex::IdSimplex& ids = vertices_id[j];
        int64_t id = m.id(tup);
        int64_t id2 = m.id(ids);
        REQUIRE(id == id2);
        REQUIRE(va.const_vector_attribute(tup) == va.const_vector_attribute(tup));
        if (is_scalar) {
            REQUIRE(va.const_scalar_attribute(tup) == va.const_scalar_attribute(tup));
        }
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

    auto& char_bacc = m.create_base_accessor(char_handle);
    auto& int64_t_bacc = m.create_base_accessor(int64_t_handle);
    auto& double_bacc = m.create_base_accessor(double_handle);

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
    // Please keep set hidden from the public unless some ugly
    // notation like these static asts exists
    {
        std::vector<char> d(size);
        std::iota(d.begin(), d.end(), char(0));
        char_bacc.set(d);
    }
    {
        std::vector<int64_t> d(size);
        std::iota(d.begin(), d.end(), int64_t(0));
        int64_t_bacc.set(d);
    }
    {
        std::vector<double> d(3 * size);
        std::iota(d.begin(), d.end(), double(0));
        double_bacc.set(d);
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
    auto& immediate_int64_t_acc = m.create_base_accessor(int64_t_handle);
    auto& immediate_double_acc = m.create_base_accessor(double_handle);

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
            auto depth = int64_t_acc.transaction_depth();
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
                using T = typename std::decay_t<decltype(va)>::Scalar;
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
        // wmtk::logger().info("Creating a scope");
        // TODO: create scope
        auto scope = m.create_scope();

        populate(m, int64_t_acc, false);
        populate(m, double_acc, false);
        check(m, int64_t_acc, false);
        check(m, double_acc, false);

        // spdlog::info("Walking out of scope");
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


TEST_CASE("attribute_clear", "[attributes]")
{
    wmtk::logger().set_level(spdlog::level::off);

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

TEST_CASE("validate_handle", "[attributes]")
{
    wmtk::logger().set_level(spdlog::level::off);

    DEBUG_TriMesh m = single_equilateral_triangle();

    auto a1 = m.register_attribute<double>("a1", wmtk::PrimitiveType::Vertex, 1);
    auto a2 = m.register_attribute<double>("a2", wmtk::PrimitiveType::Vertex, 1);
    auto a3 = m.register_attribute<double>("a3", wmtk::PrimitiveType::Vertex, 1);
    auto a4 = m.register_attribute<double>("a4", wmtk::PrimitiveType::Vertex, 2);

    m.clear_attributes({a1, a3});

    CHECK(a1.is_valid());
    CHECK_FALSE(a2.is_valid());
    CHECK(a3.is_valid());

    a1 = m.get_attribute_handle<double>("a1", wmtk::PrimitiveType::Vertex);
    a3 = m.get_attribute_handle<double>("a3", wmtk::PrimitiveType::Vertex);
    CHECK(a1.is_valid());
    CHECK(a3.is_valid());

    auto acc = m.create_accessor<double>(a1);
    acc.scalar_attribute(0) = 1;
    CHECK(acc.scalar_attribute(m.vertex_tuple_from_id(0)) == 1);
    CHECK(acc.scalar_attribute(0) == 1);

    m.clear_attributes();
    CHECK_FALSE(a1.is_valid());
    CHECK_FALSE(a2.is_valid());
    CHECK_FALSE(a3.is_valid());
    CHECK_THROWS(m.get_attribute_handle<double>("a1", wmtk::PrimitiveType::Vertex));
}

namespace {
// template magic for overload pattern
template <class... Ts>
struct overload : Ts...
{
    using Ts::operator()...;
};
template <class... Ts>
overload(Ts...) -> overload<Ts...>;

// same thing but without template magic
struct PrintVisitor
{
    Mesh& m_m;

    PrintVisitor(wmtk::Mesh& m)
        : m_m(m)
    {}

    void operator()(attribute::TypedAttributeHandle<double>& typed_handle) const
    {
        logger().info("PrintVisitor: Attribute {} is double", m_m.get_attribute_name(typed_handle));
    }

    void operator()(attribute::TypedAttributeHandle<int64_t>& typed_handle) const
    {
        logger().info(
            "PrintVisitor: Attribute {} is int64_t",
            m_m.get_attribute_name(typed_handle));
    }

    void operator()(attribute::TypedAttributeHandle<char>& typed_handle) const
    {
        logger().info("PrintVisitor: Attribute {} is char", m_m.get_attribute_name(typed_handle));
    }

    void operator()(attribute::TypedAttributeHandle<Rational>& typed_handle) const
    {
        logger().info(
            "PrintVisitor: Attribute {} is Rational",
            m_m.get_attribute_name(typed_handle));
    }
};

struct PrintVisitorT
{
    Mesh& m_m;

    PrintVisitorT(wmtk::Mesh& m)
        : m_m(m)
    {}

    template <typename HandleType>
    void operator()(HandleType& typed_handle) const
    {
        using Type = typename HandleType::Type;

        if constexpr (std::is_same_v<Type, double>) {
            logger().info(
                "PrintVisitorT: Attribute {} is double",
                m_m.get_attribute_name(typed_handle));
        }
        if constexpr (std::is_same_v<Type, int64_t>) {
            logger().info(
                "PrintVisitorT: Attribute {} is int64_t",
                m_m.get_attribute_name(typed_handle));
        }
        if constexpr (std::is_same_v<Type, char>) {
            logger().info(
                "PrintVisitorT: Attribute {} is char",
                m_m.get_attribute_name(typed_handle));
        }
    }
};

} // namespace

TEST_CASE("test_attribute_variant", "[attributes]")
{
    using namespace attribute;

    logger().set_level(spdlog::level::off);

    DEBUG_TriMesh m = single_equilateral_triangle();

    MeshAttributeHandle a1 = m.register_attribute<double>("a1", PrimitiveType::Vertex, 1);
    MeshAttributeHandle a2 = m.register_attribute<int64_t>("a2", PrimitiveType::Vertex, 1);
    MeshAttributeHandle a3 = m.register_attribute<char>("a3", PrimitiveType::Vertex, 1);

    std::vector<MeshAttributeHandle> attrs = {a1, a2, a3};

    for (MeshAttributeHandle& a : attrs) {
        std::visit(
            [&m](auto&& typed_handle) {
                // get type of the TypedAttributeHandle, e.g., TypedAttributeHandle<double>
                using HandleType = std::decay_t<decltype(typed_handle)>;
                // get attribute data type, e.g., double
                using Type = typename HandleType::Type;

                if constexpr (std::is_same_v<Type, double>) {
                    logger().info("Attribute {} is double", m.get_attribute_name(typed_handle));
                }
                if constexpr (std::is_same_v<Type, int64_t>) {
                    logger().info("Attribute {} is int64_t", m.get_attribute_name(typed_handle));
                }
                if constexpr (std::is_same_v<Type, char>) {
                    logger().info("Attribute {} is char", m.get_attribute_name(typed_handle));
                }
            },
            a.handle());
    }

    // Alternative version using the overload pattern. This requires the two template meta
    // programming stuff above this test.
    for (MeshAttributeHandle& a : attrs) {
        std::visit(
            overload{
                [&m](attribute::TypedAttributeHandle<double>& typed_handle) {
                    logger().info(
                        "Overload of double attribute {}",
                        m.get_attribute_name(typed_handle));
                },
                [&m](attribute::TypedAttributeHandle<int64_t>& typed_handle) {
                    logger().info(
                        "Overload of int64_t attribute {}",
                        m.get_attribute_name(typed_handle));
                },
                [&m](attribute::TypedAttributeHandle<char>& typed_handle) {
                    logger().info(
                        "Overload of char attribute {}",
                        m.get_attribute_name(typed_handle));
                },
                [&m](attribute::TypedAttributeHandle<Rational>& typed_handle) {
                    logger().info(
                        "Overload of Rational attribute {}",
                        m.get_attribute_name(typed_handle));
                }},
            a.handle());
    }

    // Another version, probably the easiest to understand the behavior of std::visitor
    PrintVisitor visitor(m);
    for (MeshAttributeHandle& a : attrs) {
        std::visit(visitor, a.handle());
    }

    // The same but using a templated operator
    PrintVisitorT visitor_t(m);
    for (MeshAttributeHandle& a : attrs) {
        std::visit(visitor_t, a.handle());
    }
}