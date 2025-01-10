#include "mesh_info.hpp"
#include "element_count_report.hpp"


#include "internal/MeshInfoOptions.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::mesh_info {

nlohmann::json mesh_info(const Mesh& mesh)
{
    wmtk::logger().info("The information given in this component is very limited for now. Sorry.");

    nlohmann::json report;
    report["simplex_sizes"] = element_count_report(mesh);
    return report;
}
} // namespace wmtk::components::mesh_info
