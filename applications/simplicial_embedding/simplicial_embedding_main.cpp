#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <nlohmann/json.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/simplicial_embedding/simplicial_embedding.hpp>
#include <wmtk/components/utils/resolve_path.hpp>
#include <wmtk/io/MshWriter.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/cast_attribute.hpp>
#include <wmtk/utils/primitive_range.hpp>

#include "simplicial_embedding_spec.hpp"

using namespace wmtk;
namespace fs = std::filesystem;

int main(int argc, char* argv[])
{
    CLI::App app{argv[0]};

    app.ignore_case();

    logger().info("Hello");

    fs::path json_input_file;
    app.add_option("-j, --json", json_input_file, "json specification file")
        ->required(true)
        ->check(CLI::ExistingFile);
    CLI11_PARSE(app, argc, argv);

    nlohmann::json j;
    {
        std::ifstream ifs(json_input_file);
        j = nlohmann::json::parse(ifs);

        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(j, simplicial_embedding_spec);
        if (!r) {
            logger().error("{}", spec_engine.log2str());
            return 1;
        } else {
            j = spec_engine.inject_defaults(j, simplicial_embedding_spec);
        }
    }

    const fs::path input_file =
        components::utils::resolve_paths(json_input_file, {j["root"], j["input"]});

    if (input_file.extension() != ".msh") {
        logger().error("Simplicial embedding is currently only implemented for .msh files.");
        return 1;
    }

    const std::string tag_name = j["tag_name"];

    auto mesh_in = components::input::input(input_file, false, {tag_name});
    Mesh& mesh = *mesh_in;
    wmtk::logger().info(
        "Mesh has {} vertices and {} cells",
        mesh.get_all(PrimitiveType::Vertex).size(),
        mesh.get_all(mesh.top_simplex_type()).size());

    auto tag_handle_orig = mesh.get_attribute_handle<double>(tag_name, mesh.top_simplex_type());
    auto pos_handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    std::map<PrimitiveType, std::string> tag_attribute_names;
    tag_attribute_names[PrimitiveType::Vertex] = "tag_v";
    tag_attribute_names[PrimitiveType::Edge] = "tag_e";
    tag_attribute_names[PrimitiveType::Triangle] = "tag_f";
    tag_attribute_names[PrimitiveType::Tetrahedron] = "tag_t";

    // tag_attribute_names[mesh.top_simplex_type()] = tag_name;

    // perform simplicial embedding
    components::simplicial_embedding::SimplicialEmbeddingOptions options;
    options.value = j["tag_value"];
    options.pass_through_attributes.emplace_back(pos_handle);
    options.pass_through_attributes.emplace_back(tag_handle_orig);
    for (const PrimitiveType pt :
         utils::primitive_range(PrimitiveType::Vertex, mesh.top_simplex_type())) {
        options.tag_attributes[pt] =
            mesh.register_attribute<int64_t>(tag_attribute_names[pt], pt, 1, false, -1);
    }

    // cast tag from double to int64_t
    utils::cast_attribute<int64_t>(
        tag_handle_orig,
        options.tag_attributes[mesh.top_simplex_type()]);

    if (j["embed_interfaces"]) {
        // find all facets in between different tags and on boundary
        const PrimitiveType pt_facet = (PrimitiveType)((int64_t)mesh.top_simplex_type() - 1);

        auto f_acc = mesh.create_accessor<int64_t>(options.tag_attributes[pt_facet]);
        auto c_acc = mesh.create_accessor<int64_t>(options.tag_attributes[mesh.top_simplex_type()]);

        const auto facets = mesh.get_all_id_simplex(pt_facet);
        for (const simplex::IdSimplex& f : facets) {
            if (mesh.is_boundary(pt_facet, mesh.get_tuple_from_id_simplex(f))) {
                f_acc.scalar_attribute(f) = 1;
                continue;
            }

            const auto cells = simplex::top_dimension_cofaces_tuples(mesh, mesh.get_simplex(f));
            const int64_t cell_tag = c_acc.const_scalar_attribute(cells[0]);
            for (int64_t i = 1; i < cells.size(); ++i) {
                if (c_acc.const_scalar_attribute(cells[i]) != cell_tag) {
                    f_acc.scalar_attribute(f) = 1;
                    break;
                }
            }
        }

        // set cell tags to -1
        for (const auto& c : mesh.get_all(mesh.top_simplex_type())) {
            c_acc.scalar_attribute(c) = -1;
        }
    }

    // components::output::output(mesh, "test_emb_0", "vertices");

    components::simplicial_embedding::simplicial_embedding(mesh, options);

    // components::output::output(mesh, "test_emb_1", "vertices");

    if (!j["embed_interfaces"]) {
        // cast tag back
        utils::cast_attribute<double>(
            options.tag_attributes[mesh.top_simplex_type()],
            tag_handle_orig);
    }

    wmtk::logger().info(
        "Mesh has {} vertices and {} cells",
        mesh.get_all(PrimitiveType::Vertex).size(),
        mesh.get_all(mesh.top_simplex_type()).size());

    fs::path output_file = j["output"];
    if (output_file.extension() == ".msh") {
        io::MshWriter::write(output_file, mesh, "vertices", {tag_name});
    } else {
        components::output::output(mesh, output_file, "vertices");
    }

    const std::string report = j["report"];
    if (!report.empty()) {
        nlohmann::json out_json;
        out_json["vertices"] = mesh.get_all(PrimitiveType::Vertex).size();
        out_json["edges"] = mesh.get_all(PrimitiveType::Edge).size();
        out_json["faces"] = mesh.get_all(PrimitiveType::Triangle).size();
        if (mesh.top_simplex_type() == PrimitiveType::Tetrahedron) {
            out_json["cells"] = mesh.get_all(PrimitiveType::Tetrahedron).size();
        }

        out_json["input"] = j;

        std::ofstream ofs(report);
        ofs << std::setw(4) << out_json;
    }


    return 0;
}
