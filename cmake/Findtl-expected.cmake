# Compatibility shim for generate_parameter_library exports that require the
# tl-expected CMake package even when the distro package is not installed.
#
# generate_parameter_library only needs this dependency as an interface target
# during this package's configure/generation step. Expose the common imported
# target names so its exported config can finish loading.

include(FindPackageHandleStandardArgs)

if(NOT TARGET tl::expected)
  add_library(tl::expected INTERFACE IMPORTED)
endif()

if(NOT TARGET tl-expected::tl-expected)
  add_library(tl-expected::tl-expected INTERFACE IMPORTED)
endif()

set(tl-expected_FOUND TRUE)
set(tl_expected_FOUND TRUE)

find_package_handle_standard_args(tl-expected DEFAULT_MSG tl-expected_FOUND)
