#pragma once
#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>

#include <wmtk/components/base/Paths.hpp>


namespace wmtk::components {

/**
 * @brief Perform decimation tetrahedra/triangles.
 * If you given a mesh and the tag of the top simplex. then this component will
 * preserve the tagged mesh's boundary edge and the edges incident to different
 * tagged simplices. And decimate the mesh until the all edge need to collapse
 * are large or equal the target length.
 */
void mesh_decimation(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache);

} // namespace wmtk::components
