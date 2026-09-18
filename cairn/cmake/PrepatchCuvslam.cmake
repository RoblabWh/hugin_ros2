# Build cuVSLAM from a patched copy so the submodule stays untouched.
#
# cuVSLAM only pairs cameras whose frustums overlap heavily, which rejects every pair on a wide-FOV
# 360-deg ring and leaves the camera graph empty. Lower the threshold so cuVSLAM exposes it.

set(CAIRN_CUVSLAM_SRC_CLEAN "${CAIRN_CUVSLAM_SRC}")
set(CAIRN_CUVSLAM_SRC "${CMAKE_CURRENT_BINARY_DIR}/cuvslam-src")

# Create a copy so the submodule is left untouched.
file(COPY "${CAIRN_CUVSLAM_SRC_CLEAN}/"
     DESTINATION "${CAIRN_CUVSLAM_SRC}"
     PATTERN ".git" EXCLUDE)

# Point UpdateVersion.cmake to the submodule.
set(_update_version "${CAIRN_CUVSLAM_SRC}/cmake/UpdateVersion.cmake")
if(NOT EXISTS "${_update_version}")
    message(FATAL_ERROR "cuVSLAM: ${_update_version} is missing.")
endif()
file(READ "${_update_version}" _src)
string(REPLACE "WORKING_DIRECTORY \${SOURCE_DIR}"
               "WORKING_DIRECTORY ${CAIRN_CUVSLAM_SRC_CLEAN}" _src "${_src}")
file(WRITE "${_update_version}" "${_src}")

# Lower the frustum-overlap threshold.
set(_fig_cpp "${CAIRN_CUVSLAM_SRC}/libs/camera/frustum_intersection_graph.cpp")
set(_fig_threshold_old "float intersected_num_points_ratio_threshold = 0.5;")
set(_fig_threshold_new "float intersected_num_points_ratio_threshold = 0.3;")
if(NOT EXISTS "${_fig_cpp}")
    message(FATAL_ERROR
        "cuVSLAM: ${_fig_cpp} is missing.")
endif()
file(READ "${_fig_cpp}" _src)
if(_src MATCHES "${_fig_threshold_new}")
    message(DEBUG "cuVSLAM: frustum overlap threshold already lowered")
elseif(_src MATCHES "${_fig_threshold_old}")
    string(REPLACE "${_fig_threshold_old}" "${_fig_threshold_new}" _src "${_src}")
    file(WRITE "${_fig_cpp}" "${_src}")
    message(DEBUG "cuVSLAM: lowered frustum overlap threshold (wide-FOV rig support)")
else()
    message(FATAL_ERROR "cuVSLAM: could not find the frustum overlap threshold in\n  ${_fig_cpp}")
endif()

# Re-run configure when the submodule source changes.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CAIRN_CUVSLAM_SRC_CLEAN}/libs/camera/frustum_intersection_graph.cpp")
