

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>
#include <type_traits>
#include <wmtk/operations/Operation.hpp>
#include <wmtk/utils/merkle_tree.hpp>
#include "../tools/DEBUG_Mesh.hpp"
//#include <spdlog/fmt/printf.h>

#include <catch2/catch_test_macros.hpp>
#include <polysolve/Utils.hpp>
#include <wmtk/attribute/Attribute.hpp>
#include <wmtk/attribute/CachingAccessor.hpp>
#include <wmtk/attribute/internal/AttributeTransactionStack.hpp>

#include <wmtk/PointMesh.hpp>
#include <wmtk/utils/Logger.hpp>

TEST_CASE("attribute_transaction_stack", "[attributes]")
{
    wmtk::PointMesh pm(10);

    auto vector_handle =
        pm.register_attribute_typed<double>("vector", wmtk::PrimitiveType::Vertex, 2);


    auto scalar_handle =
        pm.register_attribute_typed<int64_t>("scalar", wmtk::PrimitiveType::Vertex, 1);


    wmtk::attribute::CachingAccessor<double, 2> vector_acc(pm, vector_handle);
    wmtk::attribute::CachingAccessor<int64_t, 1> scalar_acc(pm, scalar_handle);

    wmtk::attribute::internal::AttributeTransactionStack<double> vector_ats;
    wmtk::attribute::internal::AttributeTransactionStack<int64_t> scalar_ats;


    {
        auto a = vector_acc.const_vector_attribute(0);
        auto b = vector_ats.const_vector_attribute(vector_acc, 0);

        CHECK(a.data() == b.data());
        static_assert(std::is_same_v<decltype(a), decltype(b)>);
    }
    {
        auto a = vector_acc.vector_attribute(0);
        auto b = vector_ats.vector_attribute(vector_acc, 0);

        CHECK(a.data() == b.data());
        static_assert(std::is_same_v<decltype(a), decltype(b)>);

        auto c = vector_ats.vector_attribute(vector_acc, 0);
        CHECK(c.data() == b.data());
    }
    auto run_actions = [&]() {
        for (int64_t j = 0; j < vector_acc.reserved_size(); ++j) {
            vector_acc.vector_attribute(j).setConstant(-1);
        }
        auto a = vector_acc.vector_attribute(0);
        a.setConstant(0);

        CHECK(vector_ats.buffer_end() == 0);
        CHECK(vector_ats.indices_end() == 0);

        const auto& indices = vector_ats.indices();
        auto& buffer = const_cast<std::vector<double>&>(vector_ats.buffer());
        std::fill(buffer.begin(), buffer.end(), -1);


        // first transaction is w ritten here
        auto b = vector_ats.vector_attribute(vector_acc, 0);
        // before any data is pusehd we get the same memory
        CHECK(a.data() == b.data());

        // only first 2 values
        {
            REQUIRE(indices.size() > 1);
            CHECK(indices[0].first == 0);
            CHECK(indices[0].second == 0);
            int n = 1;
            CHECK(vector_ats.buffer_end() == 2 * n);
            CHECK(vector_ats.indices_end() == n);
            REQUIRE(buffer.size() > 2 * n);
            for (int j = 0; j < n; ++j) {
                CHECK(buffer[2 * j] == j);
                CHECK(buffer[2 * j + 1] == j);
            }
            CHECK(buffer[2 * n] == -1);
        }

        a.setConstant(1);
        auto c = vector_ats.vector_attribute(vector_acc, 0);
        CHECK(c.data() == b.data());
        REQUIRE(indices.size() > 1);
        CHECK(indices[0].first == 0);
        CHECK(indices[0].second == 0);
        REQUIRE(indices.size() > 2);
        CHECK(indices[1].first == 0);
        CHECK(indices[1].second == 2);

        // only first 2 values
        {
            int n = 2;
            CHECK(vector_ats.buffer_end() == 2 * n);
            CHECK(vector_ats.indices_end() == n);
            REQUIRE(buffer.size() > 2 * n);
            for (int j = 0; j < n; ++j) {
                CHECK(buffer[2 * j] == j);
                CHECK(buffer[2 * j + 1] == j);
            }
            CHECK(buffer[2 * n] == -1);
        }

        a.setConstant(2);
        auto d = vector_ats.vector_attribute(vector_acc, 0);
        CHECK(d.data() == b.data());
        REQUIRE(indices.size() > 3);
        CHECK(indices[2].first == 0);
        CHECK(indices[2].second == 4);
        // only first 3 values
        {
            int n = 3;
            CHECK(vector_ats.buffer_end() == 2 * n);
            CHECK(vector_ats.indices_end() == n);
            REQUIRE(buffer.size() > 2 * n);
            for (int j = 0; j < n; ++j) {
                CHECK(buffer[2 * j] == j);
                CHECK(buffer[2 * j + 1] == j);
            }
            CHECK(buffer[2 * n] == -1);
        }
    };
    { // preserve_changes = true
        CHECK(vector_ats.transaction_starts() == std::vector<size_t>{});
        vector_ats.emplace();
        CHECK(vector_ats.transaction_starts() == std::vector<size_t>{0});
        run_actions();
        vector_ats.pop(vector_acc.attribute(), true);
        CHECK(vector_ats.indices_end() == 0);
        CHECK(vector_ats.transaction_starts().empty());
        std::cout << vector_acc.vector_attribute(0).transpose() << std::endl;
        std::cout << vector_acc.vector_attribute(1).transpose() << std::endl;
        std::cout << vector_acc.vector_attribute(2).transpose() << std::endl;
        std::cout << vector_acc.vector_attribute(3).transpose() << std::endl;
        CHECK((vector_acc.vector_attribute(0).array() == 2).all());
        CHECK((vector_acc.vector_attribute(1).array() == -1).all());
    }

    vector_ats = {};
    {
        CHECK(vector_ats.transaction_starts() == std::vector<size_t>{});
        vector_ats.emplace();
        CHECK(vector_ats.transaction_starts() == std::vector<size_t>{0});
        run_actions();
        {
            const auto& indices = vector_ats.indices();
            const auto& buffer = vector_ats.buffer();

            auto t_begin = vector_ats.transaction_start_begin(0);
            auto t_end = vector_ats.final_transaction_end();
            for (auto it = t_begin; it != t_end; ++it) {
                spdlog::info("{}", *it);
            }
            auto t_rend = vector_ats.transaction_start_rend(0);
            auto t_rbegin = vector_ats.final_transaction_rbegin();
            for (auto it = t_rbegin; it != t_rend; ++it) {
                spdlog::info("{}", *it);
            }
        }
        vector_ats.pop(vector_acc.attribute(), false);
        std::cout << vector_acc.vector_attribute(0).transpose() << std::endl;
        std::cout << vector_acc.vector_attribute(1).transpose() << std::endl;
        std::cout << vector_acc.vector_attribute(2).transpose() << std::endl;
        std::cout << vector_acc.vector_attribute(3).transpose() << std::endl;

        CHECK(vector_ats.indices_end() == 0);
        CHECK(vector_ats.transaction_starts().empty());
        CHECK((vector_acc.vector_attribute(0).array() == 0).all());
        CHECK((vector_acc.vector_attribute(1).array() == -1).all());
    }
    vector_ats = {};
    { // create two scopes, inner one is empty
        CHECK(vector_ats.transaction_starts() == std::vector<size_t>{});
        vector_ats.emplace();
        CHECK(vector_ats.transaction_starts() == std::vector<size_t>{0});
        run_actions();
        CHECK(vector_ats.transaction_starts() == std::vector<size_t>{0});

        vector_ats.emplace();
        CHECK(vector_ats.transaction_starts() == std::vector<size_t>{0, 3});
        CHECK(vector_ats.indices_end() == 3);

        vector_ats.pop(vector_acc.attribute(), true);
        CHECK(vector_ats.transaction_starts() == std::vector<size_t>{0});
        CHECK(vector_ats.indices_end() == 3);

        vector_ats.emplace();
        CHECK(vector_ats.transaction_starts() == std::vector<size_t>{0, 3});

        vector_ats.pop(vector_acc.attribute(), false);
        CHECK(vector_ats.transaction_starts() == std::vector<size_t>{0});
        CHECK(vector_ats.indices_end() == 3);

        // pop last scope
        vector_ats.pop(vector_acc.attribute(), false);
        CHECK(vector_ats.indices_end() == 0);
        CHECK(vector_ats.transaction_starts().empty());
        CHECK((vector_acc.vector_attribute(0).array() == 0).all());
        CHECK((vector_acc.vector_attribute(1).array() == -1).all());
    }
}

namespace {
void run(wmtk::attribute::CachingAccessor<int64_t>& accessor)
{
    accessor.scalar_attribute(0) = 3;
}
void run_nothrow_fails(wmtk::attribute::CachingAccessor<int64_t>& accessor)
{
    wmtk::attribute::AttributeManager& attribute_manager =
        wmtk::tests::DEBUG_Mesh::attribute_manager(accessor.mesh());
    wmtk::attribute::AttributeScopeHandle h(attribute_manager);
    run(accessor);

    h.mark_failed();
}

void run_with_throw_inside(wmtk::attribute::CachingAccessor<int64_t>& accessor)
{
    // oddly this should succeed because mark failed is never hit
    {
        wmtk::attribute::AttributeManager& attribute_manager =
            wmtk::tests::DEBUG_Mesh::attribute_manager(accessor.mesh());
        wmtk::attribute::AttributeScopeHandle h(attribute_manager);
        run(accessor);
        throw std::runtime_error("oh no!");
        h.mark_failed();
    }
}
void run_with_throw_outside(wmtk::attribute::CachingAccessor<int64_t>& accessor)
{
    // oddly this should succeed because mark failed is never hit
    {
        wmtk::attribute::AttributeManager& attribute_manager =
            wmtk::tests::DEBUG_Mesh::attribute_manager(accessor.mesh());
        wmtk::attribute::AttributeScopeHandle h(attribute_manager);
        run(accessor);
        h.mark_failed();
        throw std::runtime_error("oh no!");
    }
}

class Op : public wmtk::operations::Operation
{
public:
    Op(wmtk::attribute::CachingAccessor<int64_t>& acc, bool f, bool dt)
        : Operation(acc.mesh())
        , accessor(acc)
        , fail(f)
        , do_throw(dt)
    {}
    std::vector<wmtk::simplex::Simplex> execute(
        const wmtk::simplex::Simplex& simplex) final override
    {
        run(accessor);
        if (do_throw) {
            throw std::runtime_error("fff");
        }
        if (fail) {
            return {};
        } else {
            return {simplex};
        }
    }
    wmtk::PrimitiveType primitive_type() const final override
    {
        return wmtk::PrimitiveType::Vertex;
    }
    std::vector<wmtk::simplex::Simplex> unmodified_primitives(
        const wmtk::simplex::Simplex& simplex) const final override
    {
        return {simplex};
    }

private:
    wmtk::attribute::CachingAccessor<int64_t>& accessor;
    bool fail = false;
    bool do_throw = false;
};

void run_op_succ(wmtk::attribute::CachingAccessor<int64_t>& accessor)
{
    Op op(accessor, false, false);
    op(wmtk::simplex::Simplex::vertex(
        accessor.mesh(),
        accessor.mesh().get_all(wmtk::PrimitiveType::Vertex)[0]));
}
void run_op_fail(wmtk::attribute::CachingAccessor<int64_t>& accessor)
{
    Op op(accessor, true, false);
    op(wmtk::simplex::Simplex::vertex(
        accessor.mesh(),
        accessor.mesh().get_all(wmtk::PrimitiveType::Vertex)[0]));
}
void run_op_fail_throw(wmtk::attribute::CachingAccessor<int64_t>& accessor)
{
    Op op(accessor, true, true);
    op(wmtk::simplex::Simplex::vertex(
        accessor.mesh(),
        accessor.mesh().get_all(wmtk::PrimitiveType::Vertex)[0]));
}
} // namespace

TEST_CASE("attribute_transaction_throw_fail", "[attributes]")
{
    auto make_mesh = []() {
        auto pm = std::make_shared<wmtk::PointMesh>(20);

        auto handle = pm->register_attribute<int64_t>("attr", wmtk::PrimitiveType::Vertex, 1);
        wmtk::attribute::CachingAccessor<int64_t> acc(*pm, handle.as<int64_t>());
        return std::
            tuple<std::shared_ptr<wmtk::PointMesh>, wmtk::attribute::CachingAccessor<int64_t>>(
                pm,
                std::move(acc));
    };


    auto [pm_succ, acc_succ] = make_mesh();
    auto [pm_nothrow_fails, acc_nothrow_fails] = make_mesh();
    auto [pm_throw_inside, acc_throw_inside] = make_mesh();
    auto [pm_throw_outside, acc_throw_outside] = make_mesh();
    auto [pm_op_succ, acc_op_succ] = make_mesh();
    auto [pm_op_fail, acc_op_fail] = make_mesh();
    auto [pm_op_fail_throw, acc_op_fail_throw] = make_mesh();

    const size_t init_hash = pm_succ->hash();
    // make sure initial meshes are all the same
    CHECK(pm_succ->hash() == pm_nothrow_fails->hash());
    CHECK(pm_succ->hash() == pm_op_succ->hash());
    CHECK(pm_succ->hash() == pm_throw_inside->hash());
    CHECK(pm_nothrow_fails->hash() == pm_throw_outside->hash());
    CHECK(pm_nothrow_fails->hash() == pm_op_fail->hash());
    CHECK(pm_nothrow_fails->hash() == pm_op_fail_throw->hash());


    run(acc_succ);
    run_nothrow_fails(acc_nothrow_fails);
    CHECK_THROWS(run_with_throw_inside(acc_throw_inside));
    CHECK_THROWS(run_with_throw_outside(acc_throw_outside));
    run_op_succ(acc_op_succ);
    run_op_fail(acc_op_fail);
    CHECK_THROWS(run_op_fail_throw(acc_op_fail_throw));
    std::cout << wmtk::utils::merkle_tree(*pm_succ) << std::endl;
    std::cout << wmtk::utils::merkle_tree(*pm_nothrow_fails) << std::endl;

    wmtk::logger().debug(
        "Successul hash / fail hash: {} {} (init hash was {})",
        pm_succ->hash(),
        pm_nothrow_fails->hash(),
        init_hash);
    CHECK(pm_nothrow_fails->hash() == init_hash);
    CHECK(pm_succ->hash() != pm_nothrow_fails->hash());
    CHECK(pm_succ->hash() == pm_op_succ->hash());
    CHECK(pm_succ->hash() == pm_throw_inside->hash());
    CHECK(pm_nothrow_fails->hash() == pm_throw_outside->hash());
    CHECK(pm_nothrow_fails->hash() == pm_op_fail->hash());
    CHECK(pm_nothrow_fails->hash() == pm_op_fail_throw->hash());
}
