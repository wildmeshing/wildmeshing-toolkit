#include <catch2/catch_test_macros.hpp>

#include <wmtk/utils/orient_by_majority.hpp>

using namespace wmtk::utils;

TEST_CASE("orient_by_majority_consistent", "[orient][utils]")
{
    // A consistent chain turns nothing, whatever the weights.
    const std::vector<OrientationLink> links = {{0, 1, true}, {1, 2, true}, {2, 3, true}};
    const auto rep = orient_by_majority(4, links, {1, 5, 1, 1});
    CHECK(rep.n_turned == 0);
    CHECK(rep.n_non_orientable == 0);
    CHECK(rep.turn == std::vector<bool>{false, false, false, false});
}

TEST_CASE("orient_by_majority_minority_turns", "[orient][utils]")
{
    // A closed loop of four where element 2 is reversed against both its neighbours: the
    // lighter side is the one reversed, by weight rather than count.
    const std::vector<OrientationLink> links = {
        {0, 1, true},
        {1, 2, false},
        {2, 3, false},
        {3, 0, true}};
    SECTION("by count")
    {
        const auto rep = orient_by_majority(4, links, {1, 1, 1, 1});
        CHECK(rep.turn == std::vector<bool>{false, false, true, false});
        CHECK(rep.n_turned == 1);
    }
    SECTION("by weight")
    {
        const auto rep = orient_by_majority(4, links, {1, 1, 10, 1});
        CHECK(rep.turn == std::vector<bool>{true, true, false, true});
        CHECK(rep.n_turned == 3);
    }
}

TEST_CASE("orient_by_majority_components", "[orient][utils]")
{
    // Two components decide independently; an isolated element never turns.
    const std::vector<OrientationLink> links = {{0, 1, false}, {2, 3, false}};
    const auto rep = orient_by_majority(5, links, {1, 2, 3, 1, 7});
    CHECK(rep.turn == std::vector<bool>{true, false, false, true, false});
    CHECK(rep.n_turned == 2);
}

TEST_CASE("orient_by_majority_tie", "[orient][utils]")
{
    // A tie keeps the side the component's lowest element is on.
    const std::vector<OrientationLink> links = {{1, 0, false}};
    const auto rep = orient_by_majority(2, links, {1, 1});
    CHECK(rep.turn == std::vector<bool>{false, true});
}

TEST_CASE("orient_by_majority_non_orientable", "[orient][utils]")
{
    // A triangle of links with an odd number of reversals has no consistent orientation (a
    // Moebius strip): it is left as it is, and the other component still repaired.
    const std::vector<OrientationLink> links = {
        {0, 1, true},
        {1, 2, true},
        {2, 0, false},
        {3, 4, false}};
    const auto rep = orient_by_majority(5, links, {1, 1, 1, 1, 2});
    CHECK(rep.n_non_orientable == 1);
    CHECK(rep.turn == std::vector<bool>{false, false, false, true, false});
}
