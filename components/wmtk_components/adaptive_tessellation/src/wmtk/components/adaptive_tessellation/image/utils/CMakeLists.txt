set(SRC_FILES
    bicubic_interpolation.hpp
    bicubic_interpolation.cpp
    
    load_image_exr.cpp
    load_image_exr.hpp

    save_image_exr.cpp  
    save_image_exr.hpp

    sampling_utils.hpp
    sampling_utils.cpp

    SamplingParameters.hpp
)


target_sources(adaptive_tessellation PRIVATE ${SRC_FILES})