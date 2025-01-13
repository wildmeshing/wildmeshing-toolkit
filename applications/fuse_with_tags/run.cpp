#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include <numeric>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/tetwild_simplification/tetwild_simplification.hpp>
#include "fuse_eigen.hpp"

#include <set>
#include <wmtk/components/multimesh/MeshCollection.hpp>
#include "run.hpp"


wmtk::components::multimesh::NamedMultiMesh& run(
    wmtk::components::multimesh::MeshCollection& collection,
    const std::string_view& output_name,
    const std::string_view& tag_format,
    const std::string_view& position_attribute_name)
{
    auto all_meshes = collection.all_roots();


    auto mptr = fuse_eigen(collection, position_attribute_name, tag_format);
    // wmtk::components::tetwild_simplification(*mptr, std::string(position_attribute_name), 1e-3);


    return collection.emplace_mesh(*mptr, output_name);
}
