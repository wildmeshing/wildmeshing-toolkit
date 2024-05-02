
#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>
//#include <spdlog/fmt/printf.h>

#include <catch2/catch_test_macros.hpp>
#include <polysolve/Utils.hpp>
#include <wmtk/attribute/Attribute.hpp>
#include <wmtk/attribute/internal/AttributeFlatCache.hpp>
#include <wmtk/attribute/internal/AttributeMapCache.hpp>

#include <wmtk/utils/Logger.hpp>

TEST_CASE("attribute_map_cache", "[attributes]")
{
    wmtk::attribute::Attribute<int64_t> vector("vector", 2, 0, 10);


    wmtk::attribute::Attribute<int64_t> scalar("scalar", 1, 0, 10);

    wmtk::attribute::internal::AttributeMapCache<int64_t> map_cache;
    wmtk::attribute::internal::AttributeFlatCache<int64_t> flat_cache;


    wmtk::attribute::internal::AttributeMapCache<int64_t> child_map_cache;
    wmtk::attribute::internal::AttributeFlatCache<int64_t> child_flat_cache;


    std::vector<int64_t> map_data(10, 0);
    std::vector<int64_t> flat_data(10, 0);

    auto clean = [&]() {
        std::fill(map_data.begin(), map_data.end(), 0);
        std::fill(flat_data.begin(), flat_data.end(), 0);
        map_cache.clear();
        flat_cache.clear();
        child_map_cache.clear();
        child_flat_cache.clear();
    };

    auto cache = [&](int64_t index, const auto& value, bool child = false) {
        if (child) {
            child_map_cache.try_caching(index, value);
            child_flat_cache.try_caching(index, value);
        } else {
            map_cache.try_caching(index, value);
            flat_cache.try_caching(index, value);
        }
    };

    auto apply_and_check = [&](const auto& attr) {
        map_cache.apply_to(attr, map_data);
        flat_cache.apply_to(attr, flat_data);
        spdlog::warn("Map:  {}", fmt::join(map_data, ","));
        spdlog::warn("Flat: {}", fmt::join(flat_data, ","));
        CHECK(map_data == flat_data);
    };

    auto apply_child_and_check = [&](const auto& attr) {
        child_map_cache.apply_to(map_cache);
        child_flat_cache.apply_to(flat_cache);
        apply_and_check(attr);
    };


    // try caching some individual values
    cache(3, 2);
    cache(4, 1);

    // write to a vector to make sure they store the same data
    apply_and_check(scalar);
    cache(4, 0);
    // try adding one more value that overwrites another
    apply_and_check(scalar);

    // reset so we can do vector-based experiments
    clean();

    // try writing a few values
    cache(2, Eigen::Matrix<int64_t, 2, 1>::Constant(10));
    apply_and_check(vector);
    cache(1, Eigen::Matrix<int64_t, 2, 1>::Constant(3));
    apply_and_check(vector);
    cache(0, Eigen::Matrix<int64_t, 2, 1>::Constant(3));
    apply_and_check(vector);

    // overwrite an existing value
    cache(1, Eigen::Matrix<int64_t, 2, 1>::Constant(2));
    apply_and_check(vector);


    // cache to the "child cache"
    cache(1, Eigen::Matrix<int64_t, 2, 1>::Constant(-3), true);
    cache(2, Eigen::Matrix<int64_t, 2, 1>::Constant(4), true);
    cache(3, Eigen::Matrix<int64_t, 2, 1>::Constant(-2), true);
    cache(3, Eigen::Matrix<int64_t, 2, 1>::Constant(2), true);
    cache(1, Eigen::Matrix<int64_t, 2, 1>::Constant(4), true);

    // map the child to the parent, then the parent to the vector data
    apply_child_and_check(vector);


    for (int j = 0; j < 5; ++j) {
        // TODO: test get_value function
    }
}

