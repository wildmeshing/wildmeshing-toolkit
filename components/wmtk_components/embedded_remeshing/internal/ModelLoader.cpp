#include "ModelLoader.hpp"

namespace wmtk::components::internal {
void load_matrix_in_trimesh()
{
    const long embedding_tag_value = 0;
    const long input_tag_value = 1;
    const long split_tag_value = 2; // offset

    spdlog::info("Begin to load the model!");
    int grid_x, grid_y;
    grid_x = 20;
    grid_y = 20;
    Eigen::MatrixXi labels_matrix(grid_x, grid_y);
    Eigen::MatrixXi labels(grid_x * grid_y * 2, 1);
    Eigen::MatrixXd V((grid_x + 1) * (grid_y + 1), 3);
    for (int i = 0; i < grid_y + 1; ++i) {
        for (int j = 0; j < grid_x + 1; ++j) {
            V.row(i * (grid_x + 1) + j) << j, i, 0;
        }
    }
    RowVectors3l tris;
    tris.resize(2 * grid_x * grid_y, 3);
    for (int i = 0; i < grid_y; ++i) {
        for (int j = 0; j < grid_x; ++j) {
            int id0, id1, id2, id3;
            // 0       1
            // *-------*
            // | \___ 1|
            // | 0   \_|
            // *-------*
            // 2       3
            id0 = i * (grid_x + 1) + j;
            id1 = id0 + 1;
            id2 = id0 + grid_x + 1;
            id3 = id2 + 1;
            tris.row((grid_x * i + j) * 2) << id0, id2, id3;
            tris.row((grid_x * i + j) * 2 + 1) << id0, id3, id1;
            // this is a circle
            if ((i - 10) * (i - 10) + (j - 10) * (j - 10) < 25) {
                labels((grid_x * i + j) * 2, 0) = input_tag_value;
                labels((grid_x * i + j) * 2 + 1, 0) = input_tag_value;
            } else {
                labels((grid_x * i + j) * 2, 0) = embedding_tag_value;
                labels((grid_x * i + j) * 2 + 1, 0) = embedding_tag_value;
            }
        }
    }
}

void tet_divide_a(RowVectors4l& T, long x, long y, long id0, long Toffset)
{
    long id1, id2, id3, id4, id5, id6, id7;
    id1 = id0 + 1;
    id2 = id0 + x;
    id3 = id2 + 1;
    id4 = id0 + x * y;
    id5 = id4 + 1;
    id6 = id4 + x;
    id7 = id6 + 1;
    T.row(Toffset) << id0, id1, id2, id4;
    T.row(Toffset + 1) << id4, id1, id7, id5;
    T.row(Toffset + 2) << id2, id1, id7, id3;
    T.row(Toffset + 3) << id4, id2, id7, id6;
    T.row(Toffset + 4) << id4, id2, id1, id7;
}

void tet_divide_b(RowVectors4l& T, long x, long y, long id0, long Toffset)
{
    long id1, id2, id3, id4, id5, id6, id7;
    id1 = id0 + 1;
    id2 = id0 + x;
    id3 = id2 + 1;
    id4 = id0 + x * y;
    id5 = id4 + 1;
    id6 = id4 + x;
    id7 = id6 + 1;
    T.row(Toffset) << id0, id1, id3, id5;
    T.row(Toffset + 1) << id4, id0, id5, id6;
    T.row(Toffset + 2) << id6, id5, id7, id3;
    T.row(Toffset + 3) << id6, id0, id3, id2;
    T.row(Toffset + 4) << id6, id0, id5, id3;
}

void load_matrix_in_tetmesh()
{
    const long embedding_tag_value = 0;
    const long input_tag_value = 1;
    const long split_tag_value = 2; // offset

    spdlog::info("Begin to load the model!");
    int grid_x, grid_y, grid_z;
    grid_x = 20;
    grid_y = 20;
    grid_z = 20;
    Eigen::MatrixXd V((grid_x + 1) * (grid_y + 1) * (grid_z + 1), 3);
    for (int i = 0; i < grid_z + 1; ++i) {
        for (int j = 0; j < grid_y + 1; ++j) {
            for (int k = 0; k < grid_x + 1; ++k) {
                V.row(i * (grid_x + 1) * (grid_y + 1) + j * (grid_x + 1) + k) << j, i, 0;
            }
        }
    }

    // RowVectors3l tris;
    // tris.resize(2 * grid_x * grid_y, 3);
    // for (int i = 0; i < grid_y; ++i) {
    //     for (int j = 0; j < grid_x; ++j) {
    //         int id0, id1, id2, id3;
    //         // 0       1
    //         // *-------*
    //         // | \___ 1|
    //         // | 0   \_|
    //         // *-------*
    //         // 2       3
    //         id0 = i * (grid_x + 1) + j;
    //         id1 = id0 + 1;
    //         id2 = id0 + grid_x + 1;
    //         id3 = id2 + 1;
    //         tris.row((grid_x * i + j) * 2) << id0, id2, id3;
    //         tris.row((grid_x * i + j) * 2 + 1) << id0, id3, id1;
    //         // this is a circle
    //         if ((i - 10) * (i - 10) + (j - 10) * (j - 10) < 25) {
    //             labels((grid_x * i + j) * 2, 0) = input_tag_value;
    //             labels((grid_x * i + j) * 2 + 1, 0) = input_tag_value;
    //         } else {
    //             labels((grid_x * i + j) * 2, 0) = embedding_tag_value;
    //             labels((grid_x * i + j) * 2 + 1, 0) = embedding_tag_value;
    //         }
    //     }
    // }
}
} // namespace wmtk::components::internal