#include <catch2/catch.hpp>
#include <wmtk/utils/temp2.hpp>


// TEST_CASE( "Factorial of 0 is 1 (fail)", "[single-file]" ) {
//    REQUIRE( Factorial(0) == 1 );
// }

// TEST_CASE( "Factorials of 1 and higher are computed (pass)", "[single-file]" ) {
//    REQUIRE( Factorial(1) == 1 );
//    REQUIRE( Factorial(2) == 2 );
//    REQUIRE( Factorial(3) == 6 );
//    REQUIRE( Factorial(10) == 3628800 );
// }

TEST_CASE( "10 Random points are generated", "[single-file]"){
        
    using namespace wmtk;

    const std::vector<Point2D>& points = pntgen();
    REQUIRE(points.size() == 10);
}
