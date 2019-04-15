# unset version variables
unset(PROJECT_VERSION)
unset(PACKAGE_VERSION)

# Git submodule: CPR

## disabling tests for cpr as googletests cause some issue in catkin workflow:
#  "CMake Error at cpr/opt/googletest/cmake/internal_utils.cmake:145 (add_library):
#  add_library cannot create target "gtest" because another target with the
#  same name already exists.  The existing target is a shared library created
#  in source directory "/usr/src/gtest".  See documentation for policy CMP0002
#  for more details.
#  Call Stack (most recent call first):
#  cpr/opt/googletest/cmake/internal_utils.cmake:168 (cxx_library_with_type)
#  cpr/opt/googletest/CMakeLists.txt:80 (cxx_library)"
set(BUILD_CPR_TESTS OFF CACHE BOOL "Don't build cpr tests")

# build cpr as static library with position independent code
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(USE_SYSTEM_CURL "If ON, this project will look in the system paths for an installed curl library" ON)

if (USE_SYSTEM_CURL)
  find_package(CURL)
endif ()

if (NOT USE_SYSTEM_CURL OR NOT CURL_FOUND)
  set(USE_SYSTEM_CURL OFF CACHE BOOL "If ON, this project will look in the system paths for an installed curl library" FORCE)

  set(BUILD_CURL_EXE ON CACHE BOOL "Build CURL command line tool")
  set(BUILD_TESTING OFF CACHE BOOL "Build testing tree")
endif ()

add_subdirectory(cpr)

# JSON dependency

set(JSON_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
