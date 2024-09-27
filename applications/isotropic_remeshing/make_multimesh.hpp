#pragma once
#include <nlohmann/json_fwd.hpp>
#include <memory>
namespace wmtk {
    class Mesh;
}



std::shared_ptr<wmtk::Mesh> make_multimesh(wmtk::Mesh& m, const nlohmann::json& multimesh_config);

