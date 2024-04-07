#pragma once

#include <wmtk/Mesh.hpp>
#include "BackupHelper.hpp"

namespace wmtk::components::internal {

class MarchingWindow
{
public:
    MarchingWindow();
    void process();

private:
    unsigned int resolution_x, resolution_y, resolution_z;
    unsigned int window_x, window_y, window_z; // please be even
};

} // namespace wmtk::components::internal
