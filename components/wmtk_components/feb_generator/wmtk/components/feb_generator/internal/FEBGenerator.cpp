#include "FEBGenerator.hpp"
#include <time.h>
#include <fstream>
#include <set>
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
    clock_t start, end;
    start = clock();

    wmtk::attribute::MeshAttributeHandle tag_handle =
        mesh.get_attribute_handle<int64_t>("tag", PrimitiveType::Tetrahedron);
    wmtk::attribute::Accessor<int64_t> tag_acc = mesh.create_accessor<int64_t>(tag_handle);
    wmtk::attribute::MeshAttributeHandle bctag_handle =
        mesh.get_attribute_handle<int64_t>("bc_tag", PrimitiveType::Tetrahedron);
    wmtk::attribute::Accessor<int64_t> bctag_acc = mesh.create_accessor<int64_t>(bctag_handle);
    wmtk::attribute::MeshAttributeHandle id_handle =
        mesh.register_attribute<int64_t>("vertex_id", PrimitiveType::Vertex, 1);
    wmtk::attribute::Accessor<int64_t> id_acc = mesh.create_accessor<int64_t>(id_handle);
    wmtk::attribute::MeshAttributeHandle vertices_handle =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    wmtk::attribute::Accessor<double> pos_acc = mesh.create_accessor<double>(vertices_handle);

    auto v_tuple_list = mesh.get_all(PrimitiveType::Vertex);

    int64_t id = 0;
    for (const auto& t : mesh.get_all(PrimitiveType::Vertex)) {
        id_acc.scalar_attribute(t) = id;
        id++;
    }

    // get ids list
    std::set<int64_t> unique_tags;
    for (const auto& t : mesh.get_all(PrimitiveType::Tetrahedron)) {
        int64_t tag_value = tag_acc.scalar_attribute(t);
        unique_tags.insert(static_cast<int64_t>(tag_value));
    }
    std::vector<int64_t> ids_list(unique_tags.begin(), unique_tags.end());

    // begin to write feb files
    for (int64_t id : ids_list) {
        spdlog::info("current id: {}", id);
        std::vector<std::vector<int64_t>> tets;
        for (const auto& t : mesh.get_all(PrimitiveType::Tetrahedron)) {
            if (tag_acc.scalar_attribute(t) == id) {
                int64_t id0, id1, id2, id3;
                id0 = id_acc.scalar_attribute(t);
                id1 = id_acc.scalar_attribute(mesh.switch_vertex(t));
                id2 = id_acc.scalar_attribute(
                    mesh.switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Vertex}));
                id3 = id_acc.scalar_attribute(mesh.switch_tuples(
                    t,
                    {PrimitiveType::Triangle, PrimitiveType::Edge, PrimitiveType::Vertex}));
                tets.push_back(std::vector<int64_t>({id0, id1, id2, id3}));
            }
        }

        std::set<int64_t> unique_ids;
        for (const auto& tet : tets) {
            unique_ids.insert(tet.begin(), tet.end());
        }
        std::vector<int64_t> target_vertices_ids(unique_ids.begin(), unique_ids.end());

        std::string output_filename = output_folder + std::to_string(id) + ".feb";
        std::ofstream f(output_filename);

        {
            // header
            f << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\n";
            f << "<febio_spec version=\"4.0\">\n";
            f << "\t<Module type=\"solid\"/>\n";
            f << "\t<Globals>\n";
            f << "\t\t<Constants>\n";
            f << "\t\t\t<T>0</T>\n";
            f << "\t\t\t<P>0</P>\n";
            f << "\t\t\t<R>8.31446</R>\n";
            f << "\t\t\t<Fc>96485.3</Fc>\n";
            f << "\t\t</Constants>\n";
            f << "\t</Globals>\n";
            f << "\t<Mesh>\n";
        }

        {
            // original mesh
            f << "\t\t<Nodes name=\"Obj" << id << "\">\n";
            for (size_t i = 0; i < target_vertices_ids.size(); ++i) {
                int64_t v_id = target_vertices_ids[i];
                const auto point = pos_acc.const_vector_attribute(v_tuple_list[v_id]);

                f << "\t\t\t<node id=\"" << target_vertices_ids[i] + 1 << "\">" << point[0] << ","
                  << point[1] << "," << point[2] << "</node>\n";
            }
            f << "\t\t</Nodes>\n";
        }
        {
            // full surfaces
        }

        {
            // shared surfaces
        }

        {
            // custom surfaces or points
        }

        {
            f << "\t</Mesh>\n";
            f << "\t<MeshDomains>\n";
            f << "\t\t<SolidDomain name=\"Obj" << id << "\" mat=\"\"/>\n";
            f << "\t</MeshDomains>\n";
            f << "\t<Step>\n";
            f << "\t</Step>\n";
            f << "\t<Output>\n";
            f << "\t\t<plotfile type=\"febio\">\n";
            f << "\t\t\t<var type=\"displacement\"/>\n";
            f << "\t\t\t<var type=\"stress\"/>\n";
            f << "\t\t\t<var type=\"relative volume\"/>\n";
            f << "\t\t</plotfile>\n";
            f << "\t</Output>\n";
            f << "</febio_spec>\n";
        }
        f.close();
    }
    end = clock();
    spdlog::info("time: {}", (double)(end - start) / CLOCKS_PER_SEC);
}

} // namespace wmtk::components::internal
