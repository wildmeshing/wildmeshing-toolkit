#pragma once
#include <jse/jse.h>
#include <wmtk/components/topological_offset/Parameters.h>
#include <nlohmann/json.hpp>
#include <string>
#include <topological_offset_spec.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/components/simwild/expression_parser/Expression.hpp>


using namespace wmtk::components;
using ExpressionPtr = wmtk::components::simwild::expression_parser::ExpressionPtr;


namespace wmtk::components::manifold_extraction {


struct Parameters
{
    // parameters set by user
    ExpressionPtr tag_selection;
    std::set<std::string> fill_tags;
    double radius_rel;
    double radius;
    std::string output_path;
    bool debug_output;
    bool save_vtu;
    bool write_surface;

    Parameters() = default;

    Parameters(const nlohmann::json& json_params)
    {
        for (const std::string& tag : json_params["fill_tags"]) {
            if (tag == "ambient") {
                logger().warn(
                    "'ambient' tag cannot be given explicitly to fill_tags, ignoring. To "
                    "set offset to 'ambient', pass fill_tags=[].");
                continue;
            }
            fill_tags.insert(tag);
        }
        radius_rel = json_params["radius_rel"];
        radius = json_params["radius"];
        output_path = json_params["output"];
        debug_output = json_params["DEBUG_output"];
        write_surface = json_params["write_surface"];
        save_vtu = json_params["save_vtu"];
    }

    void init(const VectorXd& mins, const VectorXd& maxes)
    {
        const double diag = (maxes - mins).norm();
        if (radius > 0) {
            radius_rel = radius / diag;
        } else {
            radius = radius_rel * diag;
        }
    }

    topological_offset::Parameters generate_offset_params()
    {
        nlohmann::json offset_params;
        offset_params["application"] = "topological_offset";
        offset_params["input"] = "this doesn't matter";
        offset_params["offset_selection"] = "this should never be used.";

        // inject defaults
        const auto spec = jse::embed::wmtk_topological_offset_spec::topological_offset_spec::spec();
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(offset_params, spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        offset_params = spec_engine.inject_defaults(offset_params, spec);

        auto ret_params = topological_offset::Parameters(offset_params);
        ret_params.target_distance = radius;
        ret_params.target_distance_rel = radius_rel;
        ret_params.offset_output_tag = fill_tags;
        ret_params.debug_output = debug_output;
        ret_params.save_vtu = save_vtu;
        return ret_params;
    }
};


} // namespace wmtk::components::manifold_extraction
