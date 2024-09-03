#pragma once
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>

using json = nlohmann::json;

namespace wmtk::components::internal {
struct FullSurface
{
    std::string name;
    int64_t main_idx;
    std::vector<int64_t> exclude_ids;
};


json read_json_settings(std::string path);

bool read_fullsurface_from_json(
    json& j,
    int64_t main_idx_find,
    std::vector<std::string>& name_list,
    std::vector<std::vector<int64_t>>& exclude_ids_list);

bool read_sharedsurface_from_json(
    json& j,
    int64_t main_idx_find,
    std::vector<std::string>& name_list,
    std::vector<int64_t>& shared_idx_list);

bool read_custompart_from_json(
    json& j,
    int64_t main_idx_find,
    std::vector<std::string>& name_list,
    std::vector<std::vector<int64_t>>& shared_idx_list,
    int64_t& filter_tag);

void generate_feb_files(TetMesh& mesh, const json& j, const std::string& output_folder);

} // namespace wmtk::components::internal
