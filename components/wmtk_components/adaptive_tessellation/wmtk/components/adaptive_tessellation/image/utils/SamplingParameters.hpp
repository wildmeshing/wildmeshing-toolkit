#pragma once
namespace wmtk::components::image {
enum class SAMPLING_MODE { BICUBIC, SPLINE };
enum class IMAGE_WRAPPING_MODE { REPEAT, MIRROR_REPEAT, CLAMP_TO_EDGE };
enum class SAMPLING_METHOD { Nearest, Bilinear, Bicubic, Analytical };
} // namespace wmtk::components::image