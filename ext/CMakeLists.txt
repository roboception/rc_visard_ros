# unset the project version from parent project
remove_definitions(-DPROJECT_VERSION="${PROJECT_VERSION}")
remove_definitions(-DPACKAGE_VERSION="${PACKAGE_VERSION}")
unset(PROJECT_VERSION)
unset(PACKAGE_VERSION)
unset(abiversion)

# don't build unneeded stuff
set(BUILD_DOC OFF CACHE INTERNAL "Build docs")
set(BUILD_EXAMPLES OFF CACHE INTERNAL "Build examples")
set(BUILD_TOOLS OFF CACHE INTERNAL "Build commandline tools")

# don't build shared libs, we only need the static lib
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(rc_dynamics_api)
add_subdirectory(rc_genicam_api)

set(RC_GENICAM_API_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/rc_genicam_api ${CMAKE_CURRENT_SOURCE_DIR}/rc_genicam_api/genicam/library/CPP/include PARENT_SCOPE)
set(RC_DYNAMICS_API_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/rc_dynamics_api ${CMAKE_CURRENT_BINARY_DIR}/rc_dynamics_api/rc_dynamics_api PARENT_SCOPE)
