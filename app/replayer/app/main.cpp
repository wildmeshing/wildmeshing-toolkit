#include <wmtk/replayer/ReplayMesh.h>
#include <wmtk/replayer/Replayer.h>


#include <CLI/CLI.hpp>

#include <igl/Timer.h>
#include <igl/readOFF.h>
#include <igl/read_triangle_mesh.h>
#include <igl/writeDMAT.h>
#include <wmtk/utils/Reader.hpp>

#include <stdlib.h>
#include <chrono>
#include <cstdlib>
#include <iostream>

using namespace wmtk;
using namespace std::chrono;

int main(int argc, char** argv)
{
    std::string input_path = "";
    std::string input_point_path = "";
    std::string output_path = "out.obj";
    std::string output_point_path = "out.obj";
    int starting_position = 0;
    int count = 0;
    bool mesh_is_initial=false;

    CLI::App app{argv[0]};
    app.add_option("input", input_path, "Input mesh.")->check(CLI::ExistingFile);
    app.add_option("points", input_point_path, "Input points in barycentric coordinates.")->check(CLI::ExistingFile);
    app.add_option("output", output, "output mesh.");
    app.add_option("output_points", output_points_path, "output points.");

    app.add_option("-c, --count", count, "number of operations to run. 0 means run to end, -0 means run to beginning");
    app.add_option("-s, --start", starting_position, "which frame to start running from");
    app.add_option("--mesh-is-initial", mesh_is_initial, "Indicates if the mesh is the initial mesh, otherwise it is the start mesh");
    CLI11_PARSE(app, argc, argv);

    ReplayMesh mesh;

    ReplayMeshExecutor exec;


    int frame_count = 0;

    if(starting_position < 0) {

        starting_position = frame_count - starting_position;
    }



    if(mesh_is_initial) {
        exec.replay(start);
    }

    const int frames_to_end = frame_count - starting_position;
    const int frames_to_beginning = starting_position;

    if(count == 0) {
        count = frames_to_end;
    } else if(count == -0) {
        count = - frames_to_beginning;
    } else if(count > 0) {
        count = std::min(frames_to_end, count);
    } else {// count < 0
        count = -std::min(frames_to_beginning, -count);
    }


    if(count > 0) {
        exec.replay_forward(count);
    } else {
        exec.replay_backward(-count);
    }


    if(!output.empty()) {
    m.write_triangle_mesh(output);
    }

    if(!output.empty()) {
        p.write(output_points);
    }


    return 0;
}
