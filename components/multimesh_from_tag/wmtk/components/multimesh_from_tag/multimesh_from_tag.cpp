#include "multimesh_from_tag.hpp"

#include <deque>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/utils/get_attributes.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/utils/Logger.hpp>

#include "internal/MultiMeshFromTag.hpp"

namespace wmtk {
namespace components {

using namespace internal;

void multimesh_from_tag(
    std::shared_ptr<Mesh>& mesh_in,
    attribute::MeshAttributeHandle& substructure_label,
    int64_t substructure_value)
{
    Mesh& mesh = static_cast<Mesh&>(*mesh_in);

    MultiMeshFromTag mmft(mesh, substructure_label, substructure_value);
    mmft.compute_substructure_mesh();
    mmft.remove_soup();
}
} // namespace components
} // namespace wmtk
