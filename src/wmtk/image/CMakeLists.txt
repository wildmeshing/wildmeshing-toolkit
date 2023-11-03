set(SRC_FILES
    bicubic_interpolation.hpp
    bicubic_interpolation.cpp
    Image.cpp
    Image.hpp
    load_image_exr.cpp
    load_image_exr.h
    Sampling.hpp
    save_image_exr.cpp  
    save_image_exr.h
)
target_sources(wildmeshing_toolkit PRIVATE ${SRC_FILES})