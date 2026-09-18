# Pre-populate and patch cuNLS for CCCL 3.x (CUDA 13.1+)

include(FetchContent)

# Parse the version and hash out of the submodule
set(_cunls_cmake "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cuVSLAM/cmake/ext/cunls.cmake")
file(STRINGS "${_cunls_cmake}" _cunls_version_line REGEX "^set\\(CUNLS_VERSION")
file(STRINGS "${_cunls_cmake}" _cunls_hash_line REGEX "URL_HASH SHA256=")
string(REGEX REPLACE ".*\"([^\"]+)\".*" "\\1" CUNLS_VERSION "${_cunls_version_line}")
string(REGEX REPLACE ".*URL_HASH SHA256=([0-9a-fA-F]+).*" "\\1" CUNLS_SHA256 "${_cunls_hash_line}")
if(NOT CUNLS_VERSION OR NOT CUNLS_SHA256)
    message(FATAL_ERROR "Could not parse CUNLS_VERSION/URL_HASH from ${_cunls_cmake}.")
endif()

# Pull source tarball
FetchContent_Declare(
    cunls_prepatched
    URL https://github.com/nvidia-isaac/cuNLS/archive/refs/tags/${CUNLS_VERSION}.tar.gz
    URL_HASH SHA256=${CUNLS_SHA256}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR do-not-configure-this
)
FetchContent_MakeAvailable(cunls_prepatched)

# Patch 1: Mirror upstream cuVSLAM CMake version patch
set(_cunls_root_cmake "${cunls_prepatched_SOURCE_DIR}/CMakeLists.txt")
file(READ "${_cunls_root_cmake}" _src)
if(_src MATCHES "cmake_minimum_required\\(VERSION 3\\.24\\)")
    string(REPLACE
        "cmake_minimum_required(VERSION 3.24)"
        "cmake_minimum_required(VERSION 3.22)"
        _src "${_src}")
    file(WRITE "${_cunls_root_cmake}" "${_src}")
    message(STATUS "cuNLS: lowered cmake_minimum_required to 3.22")
endif()

# Patch 2: CCCL 3.x fix
set(_cunls_sparse_matrix "${cunls_prepatched_SOURCE_DIR}/cunls/minimizer/sparse_matrix.cu")
if(NOT EXISTS "${_cunls_sparse_matrix}")
    message(FATAL_ERROR "cuNLS: ${_cunls_sparse_matrix} not found")
endif()
file(READ "${_cunls_sparse_matrix}" _src)
if(NOT _src MATCHES "thrust/tuple\\.h")
    string(REPLACE
        "#include <thrust/iterator/zip_iterator.h>"
        "#include <thrust/iterator/zip_iterator.h>\n#include <thrust/tuple.h>"
        _src "${_src}")
    file(WRITE "${_cunls_sparse_matrix}" "${_src}")
    message(STATUS "cuNLS: patched sparse_matrix.cu to include <thrust/tuple.h> (CCCL 3.x)")
endif()
unset(_src)

set(CAIRN_CUNLS_SRC "${cunls_prepatched_SOURCE_DIR}")
message(STATUS "cuNLS: using pre-patched source (${CUNLS_VERSION}) at ${CAIRN_CUNLS_SRC}")
