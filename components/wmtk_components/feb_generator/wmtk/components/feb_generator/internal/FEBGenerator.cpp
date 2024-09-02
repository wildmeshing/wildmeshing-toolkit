#include "FEBGenerator.hpp"
#include <unordered_set>
#include "wmtk/utils/Logger.hpp"

namespace wmtk::components::internal {

json read_json_settings(std::string path)
{
    std::ifstream input_file(path);
    if (!input_file.is_open()) {
        std::cerr << "Failed to open file." << std::endl;
        return 1;
    }
    json j;
    input_file >> j;
    return j;
}

bool read_fullsurface_from_json(
    json& j,
    int64_t main_idx_find,
    std::vector<std::string>& name_list,
    std::vector<std::vector<int64_t>>& exclude_ids_list)
{
    bool found = false;
    for (const auto& full_surface : j["FullSurfaces"]) {
        std::string name = full_surface["name"];
        int64_t main_idx = full_surface["main_idx"];
        std::vector<int64_t> exclude_ids = full_surface["exclude_ids"];
        if (main_idx_find == main_idx) {
            found = true;
            name_list.push_back(name);
            exclude_ids_list.push_back(exclude_ids);
        }
    }
    return found;
}

bool read_sharedsurface_from_json(
    json& j,
    int64_t main_idx_find,
    std::vector<std::string>& name_list,
    std::vector<int64_t>& shared_idx_list)
{
    bool found = false;
    for (const auto& shared_surface : j["SharedSurfaces"]) {
        std::string name = shared_surface["name"];
        if (main_idx_find == shared_surface["main_idx"]) {
            found = true;
            name_list.push_back(shared_surface["name"]);
            shared_idx_list.push_back(shared_surface["shared_idx"]);
        } else if (main_idx_find == shared_surface["shared_idx"]) {
            found = true;
            name = name + "_shared";
            name_list.push_back(name);
            shared_idx_list.push_back(shared_surface["main_idx"]);
        }
    }
    return found;
}

bool read_custompart_from_json(
    json& j,
    int64_t main_idx_find,
    std::vector<std::string>& name_list,
    std::vector<std::vector<int64_t>>& shared_idx_list,
    int64_t& filter_tag)
{
    bool found = false;
    for (const auto& full_surface : j["CustomParts"]) {
        std::string name = full_surface["name"];
        int main_idx = full_surface["main_idx"];
        if (main_idx_find == main_idx) {
            found = true;
            name_list.push_back(name);
            std::vector<int64_t> include_ids = full_surface["exclude_ids"];
            shared_idx_list.push_back(include_ids);
            filter_tag = full_surface["filter_tag"];
        }
    }
    return found;
}

void generate_feb_files(TetMesh& mesh, const json& j, const std::string& output_folder)
{
    // get ids list
    std::unordered_set<int64_t> unique_tags;
    wmtk::attribute::MeshAttributeHandle tag_handle =
        mesh.get_attribute_handle<int64_t>("tag", PrimitiveType::Tetrahedron);
    wmtk::attribute::Accessor<int64_t> tag_acc = mesh.create_accessor<int64_t>(tag_handle);

    for (const auto& t : mesh.get_all(PrimitiveType::Tetrahedron)) {
        int64_t tag_value = tag_acc.scalar_attribute(t);
        unique_tags.insert(static_cast<int64_t>(tag_value));
    }
    std::vector<int64_t> ids_list(unique_tags.begin(), unique_tags.end());

    // begin to write feb files
    for (int64_t id : ids_list) {
        spdlog::info("current id: {}", id);

        {
            // full surfaces
        }

        {
            // shared surfaces
        }

        {
            // custom surfaces or points
        }
    }
}

} // namespace wmtk::components::internal
