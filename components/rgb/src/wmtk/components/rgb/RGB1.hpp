#include "rgb.hpp"
namespace wmtk {
class EdgeMesh;
}
namespace wmtk::components::rgb {
class RGB1 : public RGB
{
public:
    using RGB::RGB;
    virtual ~RGB1() override;
    void run() final;

private:
    EdgeMesh& mesh();
    const EdgeMesh& mesh() const;
};
} // namespace wmtk::components::rgb
