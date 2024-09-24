#include "FEBGenerator.hpp"
#include <time.h>
#include <fstream>
#include <map>
#include <set>
#include <unordered_set>
#include "FEBContactUtils.hpp"
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

// void generate_feb_files(TetMesh& mesh, const json& j, const std::string& output_folder)
//{
//     clock_t start, end;
//     start = clock();
//
//     wmtk::attribute::MeshAttributeHandle tag_handle =
//         mesh.get_attribute_handle<int64_t>("tag", PrimitiveType::Tetrahedron);
//     wmtk::attribute::Accessor<int64_t> tag_acc = mesh.create_accessor<int64_t>(tag_handle);
//     wmtk::attribute::MeshAttributeHandle bctag_handle =
//         mesh.get_attribute_handle<int64_t>("bc_tag", PrimitiveType::Tetrahedron);
//     wmtk::attribute::Accessor<int64_t> bctag_acc = mesh.create_accessor<int64_t>(bctag_handle);
//     wmtk::attribute::MeshAttributeHandle id_handle =
//         mesh.register_attribute<int64_t>("vertex_id", PrimitiveType::Vertex, 1);
//     wmtk::attribute::Accessor<int64_t> id_acc = mesh.create_accessor<int64_t>(id_handle);
//     wmtk::attribute::MeshAttributeHandle vertices_handle =
//         mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
//     wmtk::attribute::Accessor<double> pos_acc = mesh.create_accessor<double>(vertices_handle);
//
//     auto v_tuple_list = mesh.get_all(PrimitiveType::Vertex);
//
//     int64_t id = 0;
//     for (const auto& t : mesh.get_all(PrimitiveType::Vertex)) {
//         id_acc.scalar_attribute(t) = id;
//         id++;
//     }
//
//    // get ids list
//    std::set<int64_t> unique_tags;
//    for (const auto& t : mesh.get_all(PrimitiveType::Tetrahedron)) {
//        int64_t tag_value = tag_acc.scalar_attribute(t);
//        unique_tags.insert(static_cast<int64_t>(tag_value));
//    }
//    std::vector<int64_t> ids_list(unique_tags.begin(), unique_tags.end());
//
//    std::map<int64_t, std::vector<Tuple>> tet_tuple_list;
//    for (const auto& t : mesh.get_all(PrimitiveType::Tetrahedron)) {
//        int64_t tag = tag_acc.scalar_attribute(t);
//        if (tag == 0) continue;
//        tet_tuple_list[tag].push_back(t);
//    }
//
//    std::map<int64_t, std::vector<Tuple>> face_tuple_list;
//    for (const auto& t : mesh.get_all(PrimitiveType::Triangle)) {
//        int64_t tag0, tag1;
//        tag0 = tag_acc.scalar_attribute(t);
//        if (mesh.is_boundary_face(t)) {
//            continue;
//        }
//        tag1 = tag_acc.scalar_attribute(mesh.switch_tetrahedron(t));
//        if (tag0 != tag1) {
//            if (tag0 != 0) {
//                face_tuple_list[tag0].push_back(t);
//            }
//            if (tag1 != 0) {
//                face_tuple_list[tag1].push_back(t);
//            }
//        }
//    }
//
//    for (const auto& tag_list : tet_tuple_list) {
//        int64_t id = tag_list.first;
//        const std::vector<Tuple>& tets_tuples = tag_list.second;
//
//        spdlog::info("current id: {}", id);
//        std::vector<std::vector<int64_t>> tets;
//        for (const auto& t : tets_tuples) {
//            if (tag_acc.scalar_attribute(t) == id) {
//                int64_t id0, id1, id2, id3;
//                id0 = id_acc.scalar_attribute(t);
//                id1 = id_acc.scalar_attribute(mesh.switch_vertex(t));
//                id2 = id_acc.scalar_attribute(
//                    mesh.switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Vertex}));
//                id3 = id_acc.scalar_attribute(mesh.switch_tuples(
//                    t,
//                    {PrimitiveType::Triangle, PrimitiveType::Edge, PrimitiveType::Vertex}));
//                tets.push_back(std::vector<int64_t>({id0, id1, id3, id2}));
//            }
//        }
//
//        std::set<int64_t> unique_ids;
//        for (const auto& tet : tets) {
//            unique_ids.insert(tet.begin(), tet.end());
//        }
//        std::vector<int64_t> target_vertices_ids(unique_ids.begin(), unique_ids.end());
//
//        std::string output_filename = output_folder + std::to_string(id) + ".feb";
//        std::ofstream f(output_filename);
//
//        {
//            // header
//            f << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\n";
//            f << "<febio_spec version=\"4.0\">\n";
//            f << "\t<Module type=\"solid\"/>\n";
//            f << "\t<Globals>\n";
//            f << "\t\t<Constants>\n";
//            f << "\t\t\t<T>0</T>\n";
//            f << "\t\t\t<P>0</P>\n";
//            f << "\t\t\t<R>8.31446</R>\n";
//            f << "\t\t\t<Fc>96485.3</Fc>\n";
//            f << "\t\t</Constants>\n";
//            f << "\t</Globals>\n";
//            f << "\t<Mesh>\n";
//        }
//
//        {
//            // original mesh
//            // mesh vertices
//            f << "\t\t<Nodes name=\"Obj" << id << "\">\n";
//            for (size_t i = 0; i < target_vertices_ids.size(); ++i) {
//                int64_t v_id = target_vertices_ids[i];
//                const auto point = pos_acc.const_vector_attribute(v_tuple_list[v_id]);
//
//                f << "\t\t\t<node id=\"" << target_vertices_ids[i] + 1 << "\">" << point[0] << ","
//                  << point[1] << "," << point[2] << "</node>\n";
//            }
//            f << "\t\t</Nodes>\n";
//
//            // Write Elements
//            f << "\t\t<Elements type=\"tet4\" name=\"part" << id << "\">\n";
//            for (size_t i = 0; i < tets.size(); ++i) {
//                f << "\t\t\t<elem id=\"" << i + 1 << "\">" << tets[i][0] + 1 << ","
//                  << tets[i][1] + 1 << "," << tets[i][2] + 1 << "," << tets[i][3] + 1
//                  << "</elem>\n";
//            }
//            f << "\t\t</Elements>\n";
//        }
//
//        {
//            // full surfaces
//            for (const auto& item : j["FullSurfaces"]) {
//                std::string name = item["name"];
//                int64_t main_idx = item["main_idx"];
//                std::vector<int64_t> exclude_ids =
//                item["exclude_ids"].get<std::vector<int64_t>>(); std::vector<int64_t> filter_tags
//                = item["filter_tags"];
//
//                if (main_idx == id) {
//                    spdlog::info(
//                        "FullSurfaces Settings: Name: {}, Main Index: {}, Exlude IDs: {}, Filter "
//                        "Tag {}",
//                        name,
//                        main_idx,
//                        exclude_ids,
//                        filter_tags);
//                    const auto& face_tuples = face_tuple_list[main_idx];
//
//                    f << "\t\t<Surface name=\"" << name << "\">\n";
//                    int64_t cnt = 1;
//                    for (const auto& t : face_tuples) {
//                        Tuple temp_t = t;
//                        if (tag_acc.scalar_attribute(temp_t) != main_idx) {
//                            temp_t = mesh.switch_tetrahedron(t);
//                        }
//                        if (tag_acc.scalar_attribute(temp_t) != main_idx) {
//                            continue;
//                        }
//                        if (mesh.is_ccw(temp_t)) {
//                            temp_t = mesh.switch_vertex(temp_t);
//                        }
//
//                        // in exclusion set
//                        int64_t another_side_tag =
//                            tag_acc.scalar_attribute(mesh.switch_tetrahedron(temp_t));
//                        if (std::find(exclude_ids.begin(), exclude_ids.end(), another_side_tag) !=
//                                exclude_ids.end() ||
//                            std::find(
//                                filter_tags.begin(),
//                                filter_tags.end(),
//                                bctag_acc.scalar_attribute(temp_t)) == filter_tags.end()) {
//                            continue;
//                        }
//
//                        // in filter
//
//                        // write into surface
//                        f << "\t\t\t<tri3 id=\"" << cnt++ << "\">";
//                        f << id_acc.scalar_attribute(temp_t) + 1 << ","
//                          << id_acc.scalar_attribute(mesh.switch_tuples(
//                                 temp_t,
//                                 {PrimitiveType::Edge, PrimitiveType::Vertex})) +
//                                 1
//                          << "," << id_acc.scalar_attribute(mesh.switch_vertex(temp_t)) + 1;
//                        f << "</tri3>\n";
//                    }
//                    f << "\t\t</Surface>\n";
//                }
//            }
//        }
//
//        {
//            // shared surfaces
//            for (const auto& item : j["SharedSurfaces"]) {
//                std::string name = item["name"];
//                int64_t main_idx = item["main_idx"];
//                int64_t shared_idx = item["shared_idx"];
//                std::vector<int64_t> filter_tags = item["filter_tags"];
//
//                bool need_compute = false;
//                if (main_idx == id) {
//                    spdlog::info(
//                        "SharedSurfaces Settings: Name: {}, Main Index: {}, Shared Index: {}, "
//                        "Filter Tag {}",
//                        name,
//                        main_idx,
//                        shared_idx,
//                        filter_tags);
//                    need_compute = true;
//                } else if (shared_idx == id) {
//                    spdlog::info(
//                        "SharedSurfaces Settings: Name: {}, Main Index: {}, Shared Index: {}, "
//                        "Filter Tag {}",
//                        name,
//                        shared_idx,
//                        main_idx,
//                        filter_tags);
//                    need_compute = true;
//                    std::swap(main_idx, shared_idx);
//                }
//
//                if (need_compute) {
//                    const auto& face_tuples = face_tuple_list[main_idx];
//
//                    f << "\t\t<Surface name=\"" << name << "(" << main_idx << ")" << "\">\n";
//                    int64_t cnt = 1;
//                    for (const auto& t : face_tuples) {
//                        Tuple temp_t = t;
//                        if (tag_acc.scalar_attribute(temp_t) != main_idx) {
//                            temp_t = mesh.switch_tetrahedron(t);
//                        }
//                        if (tag_acc.scalar_attribute(temp_t) != main_idx) {
//                            continue;
//                        }
//                        if (mesh.is_ccw(temp_t)) {
//                            temp_t = mesh.switch_vertex(temp_t);
//                        }
//
//                        // in shared set and filter
//                        int64_t another_side_tag =
//                            tag_acc.scalar_attribute(mesh.switch_tetrahedron(temp_t));
//                        if (another_side_tag != shared_idx ||
//                            std::find(
//                                filter_tags.begin(),
//                                filter_tags.end(),
//                                bctag_acc.scalar_attribute(temp_t)) == filter_tags.end()) {
//                            continue;
//                        }
//
//                        // write into surface
//                        f << "\t\t\t<tri3 id=\"" << cnt++ << "\">";
//                        f << id_acc.scalar_attribute(temp_t) + 1 << ","
//                          << id_acc.scalar_attribute(mesh.switch_tuples(
//                                 temp_t,
//                                 {PrimitiveType::Edge, PrimitiveType::Vertex})) +
//                                 1
//                          << "," << id_acc.scalar_attribute(mesh.switch_vertex(temp_t)) + 1;
//                        f << "</tri3>\n";
//                    }
//                    f << "\t\t</Surface>\n";
//                }
//            }
//        }
//
//        {
//            for (const auto& item : j["CustomParts"]) {
//                // custom surfaces or points
//                std::string name = item["name"];
//                int64_t main_idx = item["main_idx"];
//                std::vector<int64_t> include_ids = item["include_ids"];
//                std::string type = item["custom_type"];
//                std::vector<int64_t> filter_tags = item["filter_tags"];
//
//                if (id == main_idx && type == "surface") {
//                    spdlog::info(
//                        "CustomParts Settings: Name: {}, Main Index: {}, Include Index: {}, Type "
//                        "{}, Filter Tag {}",
//                        name,
//                        main_idx,
//                        include_ids,
//                        type,
//                        filter_tags);
//
//                    const auto& face_tuples = face_tuple_list[main_idx];
//
//                    f << "\t\t<Surface name=\"" << name << "(" << main_idx << ")" << "\">\n";
//                    int64_t cnt = 1;
//                    for (const auto& t : face_tuples) {
//                        Tuple temp_t = t;
//                        if (tag_acc.scalar_attribute(temp_t) != main_idx) {
//                            temp_t = mesh.switch_tetrahedron(t);
//                        }
//                        if (tag_acc.scalar_attribute(temp_t) != main_idx) {
//                            continue;
//                        }
//                        if (mesh.is_ccw(temp_t)) {
//                            temp_t = mesh.switch_vertex(temp_t);
//                        }
//
//                        // in include set and the filter set
//                        int64_t another_side_tag =
//                            tag_acc.scalar_attribute(mesh.switch_tetrahedron(temp_t));
//                        if (std::find(include_ids.begin(), include_ids.end(), another_side_tag) ==
//                                include_ids.end() ||
//                            std::find(
//                                filter_tags.begin(),
//                                filter_tags.end(),
//                                bctag_acc.scalar_attribute(temp_t)) == filter_tags.end()) {
//                            continue;
//                        }
//
//                        // write into surface
//                        f << "\t\t\t<tri3 id=\"" << cnt++ << "\">";
//                        f << id_acc.scalar_attribute(temp_t) + 1 << ","
//                          << id_acc.scalar_attribute(mesh.switch_tuples(
//                                 temp_t,
//                                 {PrimitiveType::Edge, PrimitiveType::Vertex})) +
//                                 1
//                          << "," << id_acc.scalar_attribute(mesh.switch_vertex(temp_t)) + 1;
//                        f << "</tri3>\n";
//                    }
//                    f << "\t\t</Surface>\n";
//                } else if (id == main_idx && type == "points") {
//                    spdlog::info(
//                        "CustomParts Settings: Name: {}, Main Index: {}, Include Index: {}, Type "
//                        "{}, Filter Tag {}",
//                        name,
//                        main_idx,
//                        include_ids,
//                        type,
//                        filter_tags);
//
//                    std::set<int64_t> selected_points_set;
//                    for (const auto& t : tet_tuple_list[main_idx]) {
//                        Tuple temp_t = t;
//                        // in include set and the filter set
//                        if (std::find(
//                                filter_tags.begin(),
//                                filter_tags.end(),
//                                bctag_acc.scalar_attribute(temp_t)) == filter_tags.end()) {
//                            continue;
//                        }
//
//                        int64_t id0 = id_acc.scalar_attribute(t);
//                        int64_t id1 = id_acc.scalar_attribute(mesh.switch_vertex(t));
//                        int64_t id2 = id_acc.scalar_attribute(
//                            mesh.switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Vertex}));
//                        int64_t id3 = id_acc.scalar_attribute(mesh.switch_tuples(
//                            t,
//                            {PrimitiveType::Triangle, PrimitiveType::Edge,
//                            PrimitiveType::Vertex}));
//                        selected_points_set.insert(static_cast<int64_t>(id0));
//                        selected_points_set.insert(static_cast<int64_t>(id1));
//                        selected_points_set.insert(static_cast<int64_t>(id2));
//                        selected_points_set.insert(static_cast<int64_t>(id3));
//                    }
//                    std::vector<int64_t> selected_points(
//                        selected_points_set.begin(),
//                        selected_points_set.end());
//                    f << "\t\t<NodeSet name=\"" << name << "\">\n";
//                    if (selected_points.size() >= 1) {
//                        f << selected_points[0] + 1;
//                    }
//                    for (int64_t i = 1; i < selected_points.size(); i++) {
//                        f << "," << selected_points[i] + 1;
//                    }
//                    f << "\n\t\t</NodeSet>\n";
//                }
//            }
//        }
//
//        {
//            f << "\t</Mesh>\n";
//            f << "\t<MeshDomains>\n";
//            f << "\t\t<SolidDomain name=\"part" << id << "\" mat=\"\"/>\n";
//            f << "\t</MeshDomains>\n";
//            f << "\t<Step>\n";
//            f << "\t</Step>\n";
//            f << "\t<Output>\n";
//            f << "\t\t<plotfile type=\"febio\">\n";
//            f << "\t\t\t<var type=\"displacement\"/>\n";
//            f << "\t\t\t<var type=\"stress\"/>\n";
//            f << "\t\t\t<var type=\"relative volume\"/>\n";
//            f << "\t\t</plotfile>\n";
//            f << "\t</Output>\n";
//            f << "</febio_spec>\n";
//        }
//        f.close();
//    }
//    end = clock();
//    spdlog::info("time: {}", (double)(end - start) / CLOCKS_PER_SEC);
//}

void generate_feb_files(TetMesh& mesh, const json& j, const std::string& output_folder)
{
    bool need_merge = true;

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
    wmtk::attribute::MeshAttributeHandle id_used_handle =
        mesh.register_attribute<int64_t>("id_used", PrimitiveType::Vertex, 1);
    wmtk::attribute::Accessor<int64_t> id_used_acc = mesh.create_accessor<int64_t>(id_used_handle);
    wmtk::attribute::MeshAttributeHandle vertices_handle =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    wmtk::attribute::Accessor<double> pos_acc = mesh.create_accessor<double>(vertices_handle);

    auto v_tuple_list = mesh.get_all(PrimitiveType::Vertex);

    {
        int64_t id = 0;
        for (const auto& t : mesh.get_all(PrimitiveType::Vertex)) {
            id_acc.scalar_attribute(t) = id;
            id++;
        }
    }

    // get ids list
    std::set<int64_t> unique_tags;
    for (const auto& t : mesh.get_all(PrimitiveType::Tetrahedron)) {
        int64_t tag_value = tag_acc.scalar_attribute(t);
        unique_tags.insert(static_cast<int64_t>(tag_value));
    }
    std::vector<int64_t> ids_list(unique_tags.begin(), unique_tags.end());

    std::map<int64_t, std::vector<Tuple>> tet_tuple_list;
    for (const auto& t : mesh.get_all(PrimitiveType::Tetrahedron)) {
        int64_t tag = tag_acc.scalar_attribute(t);
        if (tag == 0) continue;
        tet_tuple_list[tag].push_back(t);
    }

    std::map<int64_t, std::vector<Tuple>> face_tuple_list;
    for (const auto& t : mesh.get_all(PrimitiveType::Triangle)) {
        int64_t tag0, tag1;
        tag0 = tag_acc.scalar_attribute(t);
        if (mesh.is_boundary_face(t)) {
            continue;
        }
        tag1 = tag_acc.scalar_attribute(mesh.switch_tetrahedron(t));
        if (tag0 != tag1) {
            if (tag0 != 0) {
                face_tuple_list[tag0].push_back(t);
            }
            if (tag1 != 0) {
                face_tuple_list[tag1].push_back(t);
            }
        }
    }

    std::string output_filename = output_folder + "output.feb";
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
    }

    {
        // materials
        f << "\t<Material>\n";
        int64_t cnt = 1;
        for (const auto& item : j["RigidMaterials"]) {
            std::string surface_name = item["name"];
            std::string rigid_material_name = surface_name + "(rigid)";
            f << "\t\t<material id=\"" << cnt << "\" name=\"" << rigid_material_name
              << "\" type=\"rigid body\">\n";
            f << "\t\t\t<density>1</density>\n";
            f << "\t\t\t<E>1</E>\n";
            f << "\t\t\t<v>0</v>\n";
            f << "\t\t</material>\n";
            cnt++;
        }
        f << "\t</Material>\n";
    }

    f << "\t<Mesh>\n";
    int64_t offset = mesh.get_all(PrimitiveType::Vertex).size();
    int64_t offset_tol = 0;
    for (const auto& tag_list : tet_tuple_list) {
        int64_t id = tag_list.first;
        const std::vector<Tuple>& tets_tuples = tag_list.second;

        spdlog::info("current id: {}", id);
        std::vector<std::vector<int64_t>> tets;
        for (const auto& t : tets_tuples) {
            if (tag_acc.scalar_attribute(t) == id) {
                int64_t id0, id1, id2, id3;
                id0 = id_acc.scalar_attribute(t);
                id1 = id_acc.scalar_attribute(mesh.switch_vertex(t));
                id2 = id_acc.scalar_attribute(
                    mesh.switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Vertex}));
                id3 = id_acc.scalar_attribute(mesh.switch_tuples(
                    t,
                    {PrimitiveType::Triangle, PrimitiveType::Edge, PrimitiveType::Vertex}));
                tets.push_back(std::vector<int64_t>({id0, id2, id1, id3}));
            }
        }

        std::set<int64_t> unique_ids;
        for (const auto& tet : tets) {
            unique_ids.insert(tet.begin(), tet.end());
        }
        std::vector<int64_t> target_vertices_ids(unique_ids.begin(), unique_ids.end());


        {
            // original mesh
            // mesh vertices
            f << "\t\t<Nodes name=\"Obj" << id << "\">\n";
            for (size_t i = 0; i < target_vertices_ids.size(); ++i) {
                int64_t v_id = target_vertices_ids[i];
                const auto point = pos_acc.const_vector_attribute(v_tuple_list[v_id]);

                if (id_used_acc.scalar_attribute(v_tuple_list[v_id]) == 0) {
                    id_used_acc.scalar_attribute(v_tuple_list[v_id]) = 1;
                } 
                else {
                    continue;
                }

                f << "\t\t\t<node id=\"" << offset_tol + target_vertices_ids[i] + 1 << "\">"
                  << point[0] << "," << point[1] << "," << point[2] << "</node>\n";
            }
            f << "\t\t</Nodes>\n";

            // Write Elements
            f << "\t\t<Elements type=\"tet4\" name=\"part" << id << "\">\n";
            for (size_t i = 0; i < tets.size(); ++i) {
                f << "\t\t\t<elem id=\"" << i + 1 << "\">" << offset_tol + tets[i][0] + 1 << ","
                  << offset_tol + tets[i][1] + 1 << "," << offset_tol + tets[i][2] + 1 << ","
                  << offset_tol + tets[i][3] + 1 << "</elem>\n";
            }
            f << "\t\t</Elements>\n";
        }

        {
            // full surfaces
            for (const auto& item : j["FullSurfaces"]) {
                std::string name = item["name"];
                int64_t main_idx = item["main_idx"];
                std::vector<int64_t> exclude_ids = item["exclude_ids"].get<std::vector<int64_t>>();
                std::vector<int64_t> filter_tags = item["filter_tags"];

                if (main_idx == id) {
                    spdlog::info(
                        "FullSurfaces Settings: Name: {}, Main Index: {}, Exlude IDs: {}, Filter "
                        "Tag {}",
                        name,
                        main_idx,
                        exclude_ids,
                        filter_tags);
                    const auto& face_tuples = face_tuple_list[main_idx];

                    f << "\t\t<Surface name=\"" << name << "\">\n";
                    int64_t cnt = 1;
                    for (const auto& t : face_tuples) {
                        Tuple temp_t = t;
                        if (tag_acc.scalar_attribute(temp_t) != main_idx) {
                            temp_t = mesh.switch_tetrahedron(t);
                        }
                        if (tag_acc.scalar_attribute(temp_t) != main_idx) {
                            continue;
                        }
                        if (!mesh.is_ccw(temp_t)) {
                            temp_t = mesh.switch_vertex(temp_t);
                        }

                        // in exclusion set
                        int64_t another_side_tag =
                            tag_acc.scalar_attribute(mesh.switch_tetrahedron(temp_t));
                        if (std::find(exclude_ids.begin(), exclude_ids.end(), another_side_tag) !=
                                exclude_ids.end() ||
                            (filter_tags.size() != 0 && std::find(
                                 filter_tags.begin(),
                                 filter_tags.end(),
                                 bctag_acc.scalar_attribute(temp_t)) == filter_tags.end())) {
                            continue;
                        }

                        // in filter

                        // is merged
                        if (need_merge && another_side_tag != 0) {
                            continue;
                        }

                        // write into surface
                        f << "\t\t\t<tri3 id=\"" << cnt++ << "\">";
                        f << offset_tol +
                                 id_acc.scalar_attribute(mesh.switch_tuples(
                                     temp_t,
                                     {PrimitiveType::Edge, PrimitiveType::Vertex})) +
                                 1
                          << "," << offset_tol + id_acc.scalar_attribute(temp_t) + 1 << ","
                          << offset_tol + id_acc.scalar_attribute(mesh.switch_vertex(temp_t)) + 1;
                        f << "</tri3>\n";
                    }
                    f << "\t\t</Surface>\n";
                }
            }
        }

        {
            // shared surfaces
            for (const auto& item : j["SharedSurfaces"]) {
                std::string name = item["name"];
                int64_t main_idx = item["main_idx"];
                int64_t shared_idx = item["shared_idx"];
                std::vector<int64_t> filter_tags = item["filter_tags"];

                bool need_compute = false;
                if (main_idx == id) {
                    spdlog::info(
                        "SharedSurfaces Settings: Name: {}, Main Index: {}, Shared Index: {}, "
                        "Filter Tag {}",
                        name,
                        main_idx,
                        shared_idx,
                        filter_tags);
                    need_compute = true;
                }
                /*
                Shared id, disable
                else if (shared_idx == id) {
                    spdlog::info(
                        "SharedSurfaces Settings: Name: {}, Main Index: {}, Shared Index: {}, "
                        "Filter Tag {}",
                        name,
                        shared_idx,
                        main_idx,
                        filter_tags);
                    need_compute = true;
                    std::swap(main_idx, shared_idx);
                }*/

                if (need_compute) {
                    // is merged
                    if (need_merge && main_idx > shared_idx) {
                        continue;
                    }

                    const auto& face_tuples = face_tuple_list[main_idx];

                    f << "\t\t<Surface name=\"" << name << "\">\n";
                    int64_t cnt = 1;
                    for (const auto& t : face_tuples) {

                        Tuple temp_t = t;
                        if (tag_acc.scalar_attribute(temp_t) != main_idx) {
                            temp_t = mesh.switch_tetrahedron(t);
                        }
                        if (tag_acc.scalar_attribute(temp_t) != main_idx) {
                            continue;
                        }
                        if (!mesh.is_ccw(temp_t)) {
                            temp_t = mesh.switch_vertex(temp_t);
                        }

                        // in shared set and filter
                        int64_t another_side_tag =
                            tag_acc.scalar_attribute(mesh.switch_tetrahedron(temp_t));
                        if (another_side_tag != shared_idx ||
                            (filter_tags.size() != 0 && std::find(
                                 filter_tags.begin(),
                                 filter_tags.end(),
                                 bctag_acc.scalar_attribute(temp_t)) == filter_tags.end())) {
                            continue;
                        }

                        // write into surface
                        f << "\t\t\t<tri3 id=\"" << cnt++ << "\">";
                        f << offset_tol +
                                 id_acc.scalar_attribute(mesh.switch_tuples(
                                     temp_t,
                                     {PrimitiveType::Edge, PrimitiveType::Vertex})) +
                                 1
                          << "," << offset_tol + id_acc.scalar_attribute(temp_t) + 1 << ","
                          << offset_tol + id_acc.scalar_attribute(mesh.switch_vertex(temp_t)) + 1;
                        f << "</tri3>\n";
                    }
                    f << "\t\t</Surface>\n";
                }
            }
        }

        {
            for (const auto& item : j["CustomParts"]) {
                // custom surfaces or points
                std::string name = item["name"];
                int64_t main_idx = item["main_idx"];
                std::vector<int64_t> include_ids = item["include_ids"];
                std::string type = item["custom_type"];
                std::vector<int64_t> filter_tags = item["filter_tags"];

                if (id == main_idx && type == "surface") {
                    spdlog::info(
                        "CustomParts Settings: Name: {}, Main Index: {}, Include Index: {}, Type "
                        "{}, Filter Tag {}",
                        name,
                        main_idx,
                        include_ids,
                        type,
                        filter_tags);

                    const auto& face_tuples = face_tuple_list[main_idx];

                    f << "\t\t<Surface name=\"" << name << "(" << main_idx << ")" << "\">\n";
                    int64_t cnt = 1;

                    for (const auto& t : face_tuples) {
                        // need merge
                        if (need_merge && main_idx > include_ids[0]) {
                            break;
                        }

                        Tuple temp_t = t;
                        if (tag_acc.scalar_attribute(temp_t) != main_idx) {
                            temp_t = mesh.switch_tetrahedron(t);
                        }
                        if (tag_acc.scalar_attribute(temp_t) != main_idx) {
                            continue;
                        }
                        if (!mesh.is_ccw(temp_t)) {
                            temp_t = mesh.switch_vertex(temp_t);
                        }

                        // in include set and the filter set
                        //int64_t another_side_tag =
                        //    tag_acc.scalar_attribute(mesh.switch_tetrahedron(temp_t));
                        int64_t another_side_tag;
                        if (main_idx == include_ids[0]) {
                            another_side_tag =
                                tag_acc.scalar_attribute(temp_t);
                        } else {
                            another_side_tag =
                                tag_acc.scalar_attribute(mesh.switch_tetrahedron(temp_t));
                        }

                        if (std::find(include_ids.begin(), include_ids.end(), another_side_tag) ==
                                include_ids.end() ||
                            (filter_tags.size() != 0 && std::find(
                                 filter_tags.begin(),
                                 filter_tags.end(),
                                 bctag_acc.scalar_attribute(temp_t)) == filter_tags.end())) {
                            continue;
                        }

                        // write into surface
                        f << "\t\t\t<tri3 id=\"" << cnt++ << "\">";
                        f << offset_tol +
                                 id_acc.scalar_attribute(mesh.switch_tuples(
                                     temp_t,
                                     {PrimitiveType::Edge, PrimitiveType::Vertex})) +
                                 1
                          << "," << offset_tol + id_acc.scalar_attribute(temp_t) + 1 << ","
                          << offset_tol + id_acc.scalar_attribute(mesh.switch_vertex(temp_t)) + 1;
                        f << "</tri3>\n";
                    }
                    f << "\t\t</Surface>\n";
                } else if (id == main_idx && type == "points") {
                    spdlog::info(
                        "CustomParts Settings: Name: {}, Main Index: {}, Include Index: {}, Type "
                        "{}, Filter Tag {}",
                        name,
                        main_idx,
                        include_ids,
                        type,
                        filter_tags);

                    std::set<int64_t> selected_points_set;
                    for (const auto& t : tet_tuple_list[main_idx]) {
                        Tuple temp_t = t;
                        // in include set and the filter set
                        if (filter_tags.size() != 0 && std::find(
                                filter_tags.begin(),
                                filter_tags.end(),
                                bctag_acc.scalar_attribute(temp_t)) == filter_tags.end()) {
                            continue;
                        }

                        int64_t id0 = id_acc.scalar_attribute(t);
                        int64_t id1 = id_acc.scalar_attribute(mesh.switch_vertex(t));
                        int64_t id2 = id_acc.scalar_attribute(
                            mesh.switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Vertex}));
                        int64_t id3 = id_acc.scalar_attribute(mesh.switch_tuples(
                            t,
                            {PrimitiveType::Triangle, PrimitiveType::Edge, PrimitiveType::Vertex}));
                        selected_points_set.insert(static_cast<int64_t>(id0));
                        selected_points_set.insert(static_cast<int64_t>(id1));
                        selected_points_set.insert(static_cast<int64_t>(id2));
                        selected_points_set.insert(static_cast<int64_t>(id3));
                    }
                    std::vector<int64_t> selected_points(
                        selected_points_set.begin(),
                        selected_points_set.end());
                    f << "\t\t<NodeSet name=\"" << name << "\">\n";
                    if (selected_points.size() >= 1) {
                        f << offset_tol + selected_points[0] + 1;
                    }
                    for (int64_t i = 1; i < selected_points.size(); i++) {
                        f << "," << offset_tol + selected_points[i] + 1;
                    }
                    f << "\n\t\t</NodeSet>\n";
                }
            }
        }

        if (!need_merge) {
            offset_tol += offset;
        }
    }

    // surface pairs part
    {
        for (const auto& item : j["NormalContacts"]) {
            std::string name = item["name"];
            std::string surface_pair_name = name + "_surface_pair";
            std::string primary = item["primary"];
            std::string secondary = item["secondary"];
            std::string contact_type = item["contact_type"];
            f << "\t\t<SurfacePair name=\"" << surface_pair_name << "\">\n";
            f << "\t\t\t<primary>" << primary << "</primary>\n";
            f << "\t\t\t<secondary>" << secondary << "</secondary>\n";
            f << "\t\t</SurfacePair>\n";
        }
    }

    // mesh end
    {
        f << "\t</Mesh>\n";
        f << "\t<MeshDomains>\n";
        for (const auto& tag_list : tet_tuple_list) {
            int64_t id = tag_list.first;
            f << "\t\t<SolidDomain name=\"part" << id << "\" mat=\"\"/>\n";
        }
        f << "\t</MeshDomains>\n";
    }

    // boundary condition
    {
        f << "\t<Boundary>\n";
        for (const auto& item : j["RigidMaterials"]) {
            std::string surface_name = item["name"];
            std::string bc_name = surface_name + "(bc)";
            std::string rigid_material_name = surface_name + "(rigid)";
            f << "\t\t<bc name=\"" << bc_name << "\" node_set=\"@surface:" << surface_name
              << "\" type=\"rigid\">\n";
            f << "\t\t\t<rb>" << rigid_material_name << "</rb>\n";
            f << "\t\t</bc>\n";
        }
        f << "\t</Boundary>\n";
    }

    // contacts
    {
        f << "\t<Contact>\n";
        for (const auto& item : j["NormalContacts"]) {
            std::string name = item["name"];
            std::string surface_pair_name = name + "_surface_pair";
            std::string primary = item["primary"];
            std::string secondary = item["secondary"];
            std::string contact_type = item["contact_type"];
            wmtk::components::internal::write_contact_template(
                f,
                contact_type,
                name,
                surface_pair_name);
        }
        f << "\t</Contact>\n";
    }

    // rigid contacts
    {
        f << "\t<Rigid>\n";
        for (const auto& item : j["RigidContacts"]) {
            std::string name = item["name"];
            std::string primary = item["primary"];
            std::string secondary = item["secondary"];
            std::string contact_type = item["contact_type"];
            std::string rigid_material1 = primary + "(rigid)";
            std::string rigid_material2 = secondary + "(rigid)";
            wmtk::components::internal::write_rigid_contact_template(
                f,
                contact_type,
                name,
                rigid_material1,
                rigid_material2);
        }
        f << "\t</Rigid>\n";
    }

    {
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
        f.close();
    }
    end = clock();
    spdlog::info("time: {}", (double)(end - start) / CLOCKS_PER_SEC);
}

} // namespace wmtk::components::internal
