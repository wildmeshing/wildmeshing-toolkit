# Flags to enable colored diagnostics on the terminal
set(MY_FLAGS
    -fdiagnostics-color=always # GCC
    -fcolor-diagnostics # Clang
)

# Flags above don't make sense for MSVC
if(MSVC)
    set(MY_FLAGS)
endif()

include(CheckCXXCompilerFlag)

foreach(FLAG IN ITEMS ${MY_FLAGS})
    string(REPLACE "=" "-" FLAG_VAR "${FLAG}")
    if(NOT DEFINED IS_SUPPORTED_${FLAG_VAR})
        check_cxx_compiler_flag("${FLAG}" IS_SUPPORTED_${FLAG_VAR})
    endif()
    if(IS_SUPPORTED_${FLAG_VAR})
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FLAG}")
        set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${FLAG}")
    endif()
endforeach()
