#include <wmtk/components/input/input.hpp>

int main(int argc, char* argv[])
{
    auto mesh = wmtk::components::input("some_mesh_file.msh");
    return 0;
}
