#pragma once
#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include <wmtk/components/utils/Paths.hpp>
#include <wmtk/io/Cache.hpp>

namespace wmtk::components {
/**
 * @name tag_intersection
 * @brief when you what to get a tag intersection between several tag attributes, then you can call
 * this function
 * @param j setting [please check the TagIntersectionOptions.hpp]
 * @param cache used for io write/read
 */
void tag_intersection(const utils::Paths& paths, const nlohmann::json& j, io::Cache& cache);

} // namespace wmtk::components
