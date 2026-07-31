if(WIN32)
    message(STATUS "Trying to find gmp+mpfr system-wide... (Windows only)")
    
    find_path(GMP_TRY_FIND_INCLUDES
    NAMES
        gmp.h
    PATHS
        ${INCLUDE_INSTALL_DIR}
    PATH_SUFFIXES
        include
    )

    find_library(GMP_TRY_FIND_LIBRARIES
    NAMES
        gmp
        libgmp-10
    PATHS
        ${LIB_INSTALL_DIR}
    PATH_SUFFIXES
        lib
    )

    find_file(GMP_TRY_FIND_RUNTIME_LIB
        NAMES
            gmp.dll
            gmp-10.dll
            libgmp-10.dll
        PATHS
            ENV GMP_DIR
            ${LIB_INSTALL_DIR}
        PATH_SUFFIXES
            bin
            lib
    )

    if(GMP_TRY_FIND_INCLUDES AND GMP_TRY_FIND_LIBRARIES AND GMP_TRY_FIND_RUNTIME_LIB)
        message(STATUS "System GMP found: \n  ${GMP_TRY_FIND_INCLUDES}\n  ${GMP_TRY_FIND_LIBRARIES}\n  ${GMP_TRY_FIND_RUNTIME_LIB}")
    else()
        message(WARNING "Third-party: downloading gmp + mpfr.")

        include(CPM)
        CPMAddPackage(
            NAME gmp_mpfr
            URL "https://github.com/CGAL/cgal/releases/download/v6.1.1/CGAL-6.1.1-win64-auxiliary-libraries-gmp-mpfr.zip"
            DOWNLOAD_ONLY YES
        )

        # For CGAL and Cork
        set(ENV{GMP_DIR} "${gmp_mpfr_SOURCE_DIR}/gmp")
        set(ENV{MPFR_DIR} "${gmp_mpfr_SOURCE_DIR}/gmp")
    endif()


    
else()
    # On Linux/macOS, gmp+mpfr should be installed system-wide
endif()