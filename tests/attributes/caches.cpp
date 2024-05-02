

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


    auto cache = [&](int64_t index, const auto& value) {
        map_cache.try_caching(index, value);
        flat_cache.try_caching(index, value);
    };
    std::vector<int64_t> map_data(10, 0);
    std::vector<int64_t> flat_data(10, 0);

    auto apply_and_check = [&](const auto& attr) {
        map_cache.apply_to(attr, map_data);
        flat_cache.apply_to(attr, flat_data);
        CHECK(map_data == flat_data);
    };

    cache(3, 2);
    cache(4, 1);

    apply_and_check(scalar);
    cache(4, 0);
    apply_and_check(scalar);

    map_cache.clear();
    flat_cache.clear();

    cache(2, Eigen::Matrix<int64_t, 2, 1>::Constant(10));
    apply_and_check(vector);
    cache(1, Eigen::Matrix<int64_t, 2, 1>::Constant(3));
    apply_and_check(vector);
    cache(0, Eigen::Matrix<int64_t, 2, 1>::Constant(3));
    apply_and_check(vector);
}

