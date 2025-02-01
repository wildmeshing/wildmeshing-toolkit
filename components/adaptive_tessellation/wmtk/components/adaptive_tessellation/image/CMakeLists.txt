set(SRC_FILES
    bicubic_interpolation.hpp
    bicubic_interpolation.cpp
    Image.cpp
    Image.hpp
    load_image_exr.cpp
    load_image_exr.hpp
    Sampling.hpp
    save_image_exr.cpp  
    save_image_exr.hpp
)


target_sources(adaptive_tessellation PRIVATE ${SRC_FILES})
