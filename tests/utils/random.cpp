#include <catch2/catch_test_macros.hpp>
#include <wmtk/utils/random_seed.hpp>
#include <Eigen/Dense>


TEST_CASE("random", "[random]")
{

    wmtk::utils::set_random_seed(20);
    std::mt19937 mt_a(wmtk::utils::get_random_seed());

    Eigen::MatrixXd A = Eigen::MatrixXd::Random(20,20);

    std::mt19937 mt_b(wmtk::utils::get_random_seed());
    Eigen::MatrixXd B = Eigen::MatrixXd::Random(20,20);

    wmtk::utils::set_random_seed(20);
    std::mt19937 mt_ap(wmtk::utils::get_random_seed());
    Eigen::MatrixXd Ap = Eigen::MatrixXd::Random(20,20);



    for(int j = 0; j < 20; ++j) {
        uint32_t a = mt_a();
        uint32_t b = mt_b();
        uint32_t ap = mt_ap();
        CHECK(a == ap);
        // theoretically a could == b, but are choosing a seed where this isn't so
        CHECK(a != b);
    }

    CHECK(A == Ap);
    CHECK(B != A);
}
