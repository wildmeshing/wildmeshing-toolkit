#include <stdio.h>
#include <stdlib.h>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplit.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk_components/embedded_remeshing/internal/ModelLoader.hpp>
#include <wmtk_components/mesh_info/mesh_info.hpp>
#include <wmtk_components/regular_space/internal/RegularSpace.hpp>
#include <wmtk_components/regular_space/regular_space.hpp>
#include "wmtk/../../tests/tools/DEBUG_TetMesh.hpp"
#include "wmtk/../../tests/tools/DEBUG_TriMesh.hpp"
#include "wmtk/../../tests/tools/TetMesh_examples.hpp"
#include "wmtk/../../tests/tools/TriMesh_examples.hpp"

using json = nlohmann::json;
using namespace wmtk;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("matrix_load", "[components][.]")
{
    SECTION("trimesh_load")
    {
        std::vector<std::vector<long>> labels;
        for (long j = 0; j < 22; ++j) {
            std::vector<long> line;
            line.reserve(20);
            for (long i = 0; i < 20; ++i) {
                if ((i - 10) * (i - 10) + (j - 11) * (j - 11) < 36) {
                    line.push_back(1);
                } else {
                    line.push_back(0);
                }
            }
            labels.push_back(line);
        }
        TriMesh mesh;
        wmtk::components::internal::load_matrix_in_trimesh(mesh, labels);
        if (true) {
            ParaviewWriter
                writer(data_dir / "trimesh_matrix_load", "position", mesh, true, true, true, false);
            mesh.serialize(writer);
        }
    }
    SECTION("tetmesh_load")
    {
        std::vector<std::vector<std::vector<long>>> labels;
        for (long k = 0; k < 24; ++k) {
            std::vector<std::vector<long>> layer;
            for (long j = 0; j < 22; ++j) {
                std::vector<long> line;
                line.reserve(20);
                for (long i = 0; i < 20; ++i) {
                    if ((i - 10) * (i - 10) + (j - 11) * (j - 11) + (k - 12) * (k - 12) < 36) {
                        line.push_back(1);
                    } else {
                        line.push_back(0);
                    }
                }
                layer.push_back(line);
            }
            labels.push_back(layer);
        }
        TetMesh mesh;
        wmtk::components::internal::load_matrix_in_tetmesh(mesh, labels);
        if (true) {
            ParaviewWriter
                writer(data_dir / "tetmesh_matrix_load", "position", mesh, true, true, true, true);
            mesh.serialize(writer);
        }
    }
}