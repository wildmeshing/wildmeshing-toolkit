# PolyFEM (https://github.com/polyfem/polyfem)
# License: MIT
#
# polyfem's sources only (polyfem_SOURCE_DIR, polyfem_BINARY_DIR). recipes/polyfem.cmake builds
# them; recipes/libigl.cmake reads polyfem's libigl patches from them, before polyfem is added.

include(CPM)
# uday/imagesim: polyfem main (2b302cb749) and the image-simulation work.
CPMAddPackage(
    NAME polyfem
    GITHUB_REPOSITORY polyfem/polyfem
    GIT_TAG aa17c219c3366429573d400efcbcfdabe06245cd
    DOWNLOAD_ONLY YES
)
