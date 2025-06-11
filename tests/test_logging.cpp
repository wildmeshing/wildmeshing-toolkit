
#include <catch2/catch_test_macros.hpp>

#include <wmtk/utils/json_sink.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>

TEST_CASE("json_sink", "[io][logging]")
{
    std::filesystem::path log_path("./test_json_logger.log");
    auto logger = wmtk::make_json_file_logger("test", log_path);

    nlohmann::json input;
    input["a"] = 3;
    input["b"]["c"] = std::array<int, 3>{{0, 1, 2}};
    input["b"]["d"] = "welp";

    logger->info(input.dump());
    // TODO: check output to stdio is fine
    // TODO: auto check that these contents are same as file
}
