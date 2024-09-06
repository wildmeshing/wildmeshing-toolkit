#include "make_mesh.hpp"
#include "GridOptions.hpp"
#include "Grid2Options.hpp"
#include "Grid3Options.hpp"



namespace wmtk::components::procedural {
std::shared_ptr<Mesh> make_mesh(const GridOptions& m) {
    return std::visit([](const auto& o)  -> std::shared_ptr<Mesh>{
            return make_mesh(o);
            },m.opts);

}
}
