# Build thirdparty/cuVSLAM as an ExternalProject and expose it as the IMPORTED target `cuvslam`.

include(ExternalProject)

set(CAIRN_CUDA_ARCH "native" CACHE STRING "CUDA architectures for the cuVSLAM build")

set(CAIRN_CUVSLAM_SRC "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cuVSLAM")
set(CAIRN_CUVSLAM_BUILD "${CMAKE_CURRENT_BINARY_DIR}/cuvslam-build")
set(CAIRN_CUVSLAM_LIB "${CAIRN_CUVSLAM_BUILD}/bin/libcuvslam.so")

# Frustum-overlap threshold fix
include(${CMAKE_CURRENT_LIST_DIR}/PrepatchCuvslam.cmake)

# CCCL 3.x cuNLS fix
include(${CMAKE_CURRENT_LIST_DIR}/PrepatchCunls.cmake)

ExternalProject_Add(cuvslam_ext
    SOURCE_DIR "${CAIRN_CUVSLAM_SRC}"
    BINARY_DIR "${CAIRN_CUVSLAM_BUILD}"
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=Release
        -DCMAKE_CUDA_ARCHITECTURES=${CAIRN_CUDA_ARCH}
        -DFETCHCONTENT_SOURCE_DIR_CUNLS=${CAIRN_CUNLS_SRC}
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    INSTALL_COMMAND ""
    BUILD_ALWAYS ON
    BUILD_BYPRODUCTS "${CAIRN_CUVSLAM_LIB}"
    USES_TERMINAL_CONFIGURE ON
    USES_TERMINAL_BUILD ON
)

add_library(cuvslam SHARED IMPORTED GLOBAL)
set_target_properties(cuvslam PROPERTIES
    IMPORTED_LOCATION "${CAIRN_CUVSLAM_LIB}"
    IMPORTED_NO_SONAME TRUE
    INTERFACE_INCLUDE_DIRECTORIES "${CAIRN_CUVSLAM_SRC}/libs"
)
