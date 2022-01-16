// Local library code
#include <remeshing/dummy.h>

#include <wmtk/utils/Logger.hpp>

// Third-party include
#include <CLI/CLI.hpp>

// System include
#include <iostream>


int main(int argc, char const* argv[])
{
    struct
    {
        std::string input;
        double ratio = 1;
    } args;

    CLI::App app{argv[0]};
    // app.option_defaults()->always_capture_default();
    app.add_option("input", args.input, "Input mesh.");
    app.add_option("-r,--ratio", args.ratio, "Ratio.");
    CLI11_PARSE(app, argc, argv);

    wmtk::logger().info("Ratio: {}", args.ratio);

    return 0;
}
