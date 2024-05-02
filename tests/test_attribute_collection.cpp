#include <igl/Timer.h>
#include <tbb/concurrent_vector.h>
#include <catch2/catch.hpp>
#include <wmtk/AttributeCollection.hpp>

TEST_CASE("attribute_performance", "[attribute]")
{
    using namespace wmtk;
    tbb::concurrent_vector<int> raw_vector;
    AttributeCollection<int> attribute_collection;

    double time = 0;
    igl::Timer timer;

    size_t N = 20;
    size_t iter = 1000000;

    raw_vector.resize(N);
    attribute_collection.resize(N);

    timer.start();
    for (size_t k = 0; k < iter; ++k) {
        for (size_t i = 0; i < N; ++i) {
            raw_vector[i] = 1;
        }
    }
    std::cout << "raw tbb vector write: " << timer.getElapsedTimeInMilliSec() << std::endl;


    attribute_collection.begin_protect();
    // attribute_collection.end_protect();
    timer.start();
    for (size_t k = 0; k < iter; ++k) {
        for (size_t i = 0; i < N; ++i) {
            attribute_collection[i] = 1;
        }
    }
    std::cout << "attribute collection write: " << timer.getElapsedTimeInMilliSec() << std::endl;
}