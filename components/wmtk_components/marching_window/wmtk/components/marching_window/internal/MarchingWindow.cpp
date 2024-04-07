#include "MarchingWindow.hpp"

namespace wmtk::components::internal {
MarchingWindow::MarchingWindow() {}

void MarchingWindow::process()
{
    // set up
    unsigned int cur_x, cur_y, cur_z;
    cur_x = cur_y = cur_z = 0;
    unsigned int step_x, step_y, step_z;
    step_x = window_x / 2 + 1;
    step_y = window_y / 2 + 1;
    step_z = window_z / 2 + 1;

    for (unsigned int cur_z = 0; cur_z < resolution_z; cur_z += step_z) {
        // z
        for (unsigned int cur_y = 0; cur_y < resolution_y; cur_y += step_y) {
            // y
            for (unsigned int cur_x = 0; cur_x < resolution_x; cur_x += step_x) {
                // x
            }
        }
    }
}

} // namespace wmtk::components::internal
