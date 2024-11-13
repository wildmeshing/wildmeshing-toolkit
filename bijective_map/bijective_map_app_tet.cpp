#include <CLI/CLI.hpp>
#include <filesystem>
#include <iostream>
#include <sstream>

// wmtk
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/orient.hpp>
using namespace wmtk;

// igl, nothing for now

// applications

int main(int argc, char** argv)
{
    CLI::App app{"bijective_map_app"};
    std::string application_name = "back";
    app.add_option("-a, --app", application_name, "Application name");

    CLI11_PARSE(app, argc, argv);

    std::cout << "Application name: " << application_name << std::endl;


    // TODO:
    // 1. read the input mesh, or just dump it to something that paraview can read?
    // 2. figure out how to read the outputmesh in vtu format
    // 3. for the first experiment, nee
    return 0;
}